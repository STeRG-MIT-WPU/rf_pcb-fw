# RF board handoff: RF board <-> FC, ground station, radios

Source: `rf_pcb-fw` main, commit `8c783e7`, 29 Oct 2026.
Files of record: `Core/Src/freertos.c` (all tasks and tunables), `Core/Src/telemetry.c` /
`Core/Inc/telemetry.h` (FC frame parse/format), `README.md` (pins, baud rates).
When this file and the code disagree, the code wins. Re-read the `#define` block at the top
of `freertos.c` first.

Purpose: the RF board sits between the flight computer (FC) and three radios. It parses the FC
telemetry stream, re-sends it on each radio at that radio's own rate, relays camera image lines,
and forwards ground-station telecommands back to the FC. This file lists what goes in and out
of every port, the rules the other sides must follow, and the values that must be set before
flight.

```
            USART3 921600                 UART5 115200   E22-900T22D LoRa   telemetry every 3 s
 FC  <------------------------>  RF board <------------> 
   telemetry 10 Hz, $IMG lines            USART2 115200  XBee Pro           telemetry 5 Hz + image lines
   <- telecommands, IMGACK                UART4  921600  EC200U GSM/LTE     MQTT 15 Hz + image + acks, SMS / 60 s
```

MCU: STM32L4R5VIT6, 120 MHz, FreeRTOS (CMSIS-RTOS v2). Image ring (270 KB) lives in RAM3.

## 1. Ports

| Port | Peripheral | Baud | RX method | Other side |
|---|---|---|---|---|
| FC | USART3, PC4 TX / PC5 RX | 921600 8N1 | 2 KB circular DMA + IDLE | FC USART3 (PD8 TX / PD9 RX) |
| LoRa | UART5, PC12 TX / PD2 RX | 115200 8N1 | IDLE interrupt, 120 B buffer | E22-900T22D, transparent mode (M0/M1 tied on PCB) |
| XBee | USART2, PA2 TX / PA3 RX | 115200 8N1 | IDLE interrupt, 120 B buffer | XBee Pro, transparent mode; PA5 nCTS in, PA6 nRTS out (held low) |
| GSM | UART4, PA0 TX / PA1 RX | 921600 8N1 | IDLE interrupt, 128 B chunks into 1 KB accumulator | EC200U AT port |

No hardware flow control anywhere. 3.3 V logic. The firmware never changes a module's baud
rate or radio settings: every module must be pre-configured (section 7).

## 2. FC link (USART3)

### 2.1 FC -> RF board

**Telemetry line**, 10 Hz, exactly as the FC `TELECOMMAND_HANDOFF.md` / FC README defines it:

```
$,TEAM_ID,TIME,PKT,ACC(x|y|z),GYR(x|y|z),MAG(x|y|z),EUL(r|p|y),ALT,PRES,TEMP,VEL(x|y|z),
GNSS_TIME,LAT,LON,GNSS_ALT,SATS,GNSS_VEL(x|y),CURRENT,VOLT,BATT%,STATE,EFLAG,PROP_DEPLOY,RW_STATUS,#\r\n
```

- 26 comma fields, triplets joined with `|`, `EFLAG` is hex, `PROP_DEPLOY` 0/1/2, `RW_STATUS` 0/1.
- A line that does not parse to this exact field count is dropped silently. Adding or removing
  a field on the FC without updating `telemetry.c` stops every radio.
- Any FC line up to 1407 characters is buffered; longer lines are discarded.

**Image lines** `$IMGSTART`, `$IMG`, `$IMGEND` are stored verbatim (section 4).
Anything else starting with `$` is ignored. Lines not starting with `$` are ignored.

### 2.2 RF board -> FC

- Telecommand lines, upper-case, single spaces, CRLF, printable ASCII, max 118 characters plus CRLF,
  one line per 20 ms (section 3). Never a line starting with `$`.
- `IMGACK,<id>,OK\r\n` or `IMGACK,<id>,FAIL\r\n` (section 4).
- Nothing else. The RF board never echoes and never sends telemetry back.

## 3. Telecommands

### 3.1 Sources

| Source | How a command arrives |
|---|---|
| LoRa | Raw bytes on UART5. One IDLE gap = one command. Send the whole line in one burst. |
| XBee | Raw bytes on USART2, same rule. A line starting with `AT` is an XBee AT telecommand (section 3.5), not an FC command. |
| MQTT | Payload of a message on `MQTT_TOPIC_CMD` (`cansat/011/cmd`), QoS 0. One command per message. |
| SMS | First line of a text from `SMS_CMD_FROM` (`+919373986194`). Texts from other numbers are discarded. Set `SMS_CMD_FROM` to `""` to accept any sender. |

Accepted forms, either works on every source:

- Raw FC console words: `BUZZ ON`, `CAPTURE`, `ARM`, `DEPLOY`.
- Ground-station frame `CMD,<team>,<verb>,<arg>,<arg>` which becomes `<VERB> <ARG> <ARG>`.

### 3.2 What the RF board does with a line

1. Upper-cases, drops non-printable bytes, collapses repeated spaces, trims. Empty result,
   first character `$`, or longer than 118 characters: dropped, ack `BAD`.
2. `IMG LINK MQTT` is refused (`BAD`): its 13.8 KB image lines do not fit the relay ring.
   `IMG LINK XBEE` (1024 raw bytes per line, FC default) is the only supported mode.
3. `ARM` is consumed on the RF board, never sent to the FC. It opens a 60 s window, ack `ARMED`.
4. While no window is open, lines whose first word is `RESET`, `BURN`, `PAYLOAD`, `DEPLOY`,
   `PARA`, `STATE`, `PERSIST`, `SD`, `BENCH`, `DIAG` or `SIM` are blocked, ack `ARM?`.
   Sequence for an irreversible action: send `ARM`, wait for `ARMED`, then the command.
5. Everything else is queued (8 deep) and sent to the FC with CRLF, at most one line every
   20 ms, ack `OK`.
6. After `GYRO CAL` the queue is held 13 s and after `SD CLEAR` 3 s, because the FC command
   task is blocked meanwhile. Queued commands go out afterwards in order.

### 3.3 Acks

The FC never replies on USART3, so the RF board publishes its own verdict on `MQTT_TOPIC_ACK`
(`cansat/011/ack`), one message per received line:

| Ack | Meaning |
|---|---|
| `OK <line>` | forwarded to the FC as shown |
| `ARMED` | ARM window opened |
| `ARM? <line>` | blocked, send `ARM` first |
| `BAD <line>` | unusable line, not forwarded |

Acks go only to MQTT, including for LoRa, XBee and SMS commands. Without a GSM link there are
no acks. Confirm the actual effect through `STATE` and `EFLAG` in the telemetry.

### 3.4 LoRa timing (half duplex)

The E22 cannot receive while it transmits. At 2.4 kbps air rate one ~280 byte telemetry frame is
about 1.15 s on air. The RF board sends it every 3 s as a 240 byte chunk, a 1 s gap, then the
rest, so the module listens roughly 55 % of the time. For reliable telecommands the ground
station should send its line right after it receives a LoRa telemetry line, and send it in one
burst. A line arriving while the E22 is transmitting is lost.

### 3.5 XBee AT telecommands (XBee source only)

Ported from the `rf 4` reference firmware. A USART2 line that starts with `AT` configures the XBee
module on the RF board instead of going to the FC:

1. Commands that change the RF link (`ATCH`, `ATID`, `ATDH`, `ATDL`, `ATZS`, `ATEE`, `ATEO`,
   `ATKY`) are answered `RESP|<cmd>|OK|ACCEPTED` immediately, because the link may not work after.
2. XBee telemetry and image lines pause. After 1.1 s of silence the board sends `+++`, waits for
   `OK` (2 s), sends `<cmd>\r`, reads the first reply line, sends `ATWR\r` when a value was set
   (not for `ATWR`, `ATCN`, `ATFR`, `ATRE` or plain queries), then `ATCN\r` and waits 100 ms.
3. Reply on XBee: `RESP|<cmd>|OK|<first reply line>` or `RESP|<cmd>|ERROR|<detail>` with detail
   `NO_CMD_MODE`, `TIMEOUT`, `ATWR_FAIL`, `UART` or the module's error text. No reply after an
   RF-link change (already answered) or `ATFR` (module reboots).
4. One command at a time. AT lines arriving while one is queued or running are dropped. The
   whole sequence takes 1.5 to 3 s, during which XBee telemetry stops and LoRa/GSM continue.

Only the first four letters of the command appear in the `RESP` line. There is no AT path for the
LoRa E22 (M0/M1 fixed on the PCB).

## 4. Image relay

The FC sends a captured image as lines, relayed verbatim (with CRLF) on XBee and on
`MQTT_TOPIC_IMG`:

```
$IMGSTART,<id>,<size>,<total>*hh
$IMG,<id>,<seq>,<total>,<len>,<base64>*hh        1024 raw bytes per line, ~1400 characters
$IMGEND,<id>,<total>,<size>*hh
```

- Relay ring: 192 lines, ~190 KB of image. Slot = seq modulo 192; full when the slower of the
  two readers (XBee, GSM) is 192 lines behind.
- XBee sends one image line in the spare part of each 200 ms slot (~5 KB/s). The GSM task
  publishes one line per 66 ms period (~15 lines/s, ~15 KB/s on a good network). When the GSM
  link is down its reader is parked at the write pointer so it never holds XBee back.
- `IMGACK,<id>,OK` is sent to the FC once both readers passed `$IMGEND`.
  `IMGACK,<id>,FAIL` is sent as soon as a line is too long (over 1407 characters) or the ring
  overflowed. The FC then resends.
- Checksums are not verified on the RF board. A line whose MQTT publish fails is dropped for MQTT
  only; XBee is unaffected.

## 5. Radio outputs

Every radio sends the same line layout as section 2.1, re-formatted from the parsed values with
its own packet counter in `PKT`. Nothing is sent until the first valid FC line has arrived.

| Radio | Rate | Payload |
|---|---|---|
| LoRa | every 3000 ms | telemetry line, 240 B then rest after 1 s |
| XBee | every 200 ms | telemetry line, then image lines; 256 B pieces, nCTS checked before each (waits up to 50 ms, then sends anyway) |
| MQTT `cansat/011/tlm` | every 66 ms (~15 Hz) | telemetry line, QoS 0 |
| MQTT `cansat/011/img` | one line per period | image line |
| MQTT `cansat/011/ack` | as produced | ack strings of section 3.3 |
| SMS to `SMS_NUMBER` | every 60 s | `T+<s>s GNSS <time> <lat>,<lon> alt <m>m sats <n>` |

### 5.1 GNSS source

The EC200U's own GNSS is switched on (`AT+QGPS=1`) and polled once per second (`AT+QGPSLOC=2`).
When it reports more satellites than the FC's L89, its `GNSS_TIME`, `LAT`, `LON`, `GNSS_ALT`,
`SATS` and `GNSS_VEL` replace the FC values in every outgoing packet on every radio. Tie or no
modem fix: FC values. All other fields are always the FC's. Ground software cannot tell which
receiver a packet came from except through the satellite count.

## 6. GSM / MQTT session

Bring-up sequence on UART4 (each with a timeout, retried forever):
`AT`, `ATE0`, `AT+QGPS=1`, `AT+CPIN?` (needs `READY`), `AT+CREG?` until registered,
`AT+CMGF=1`, `AT+CNMI=2,2,0,0,0`, `AT+QICSGP=1,1,"<APN>","","",1`, `AT+QIACT=1`,
then `AT+QMTCLOSE=0`, `AT+QMTCFG="version",0,4`, `AT+QMTOPEN=0,"<broker>",<port>`,
`AT+QMTCONN=0,"<client id>"`, `AT+QMTSUB=0,1,"<cmd topic>",0`.

- Publishes use `AT+QMTPUBEX`, QoS 0, retain 0. Eight consecutive refused publishes, or a
  `+QMTSTAT` URC, drop the session and the board reconnects (broker retry every 3 s, full modem
  bring-up again every 5th miss).
- Incoming SMS and MQTT messages arrive as URCs (`+CMT`, `+QMTRECV`) and are handled on every
  loop pass. The modem does not store SMS.
- Expect telemetry gaps of a few seconds on MQTT around reconnects and around each SMS send
  (up to 8 s wait for `+CMGS`).

## 7. Values that must be set before flight

All are `#define`s at the top of `Core/Src/freertos.c`:

| Define | Current value | Set to |
|---|---|---|
| `GSM_APN` | `airtelgprs.com` | the SIM's APN |
| `MQTT_BROKER`, `MQTT_PORT` | `broker.hivemq.com`, 1883 | the mission broker |
| `MQTT_CLIENT_ID` | `cansat_rf_011` | unique per board |
| `MQTT_TOPIC_TLM/CMD/IMG/ACK` | `cansat/011/...` | agree with the ground station |
| `SMS_NUMBER` | `+916375605728` | ground phone that receives the 60 s SMS |
| `SMS_CMD_FROM` | `+919373986194` | ground phone allowed to send SMS commands, in the exact format the operator delivers in `+CMT` (check once on the bench) |

Module configuration done off-board, not by this firmware:

- E22-900T22D: UART 115200 8N1, transparent mode, air rate 2.4 kbps, address 138, NET ID 11,
  channel 17 (see `CanSat_LoRa_HOWTO.txt` in the rf_txtest folder). Ground E22 must match. The
  RF board cannot reconfigure it because M0/M1 are tied on the PCB.
- XBee Pro: 115200 8N1, transparent (AT) mode, same PAN/channel as the ground XBee, CTS flow
  control enabled on its DIO7 (the board polls nCTS). No API mode.
- EC200U: AT port at 921600, SIM with data plan, variant with GNSS.
- FC: USART3 at 921600, `IMG LINK XBEE` (default).

## 8. Known limits and open items

1. **Half-duplex LoRa.** Commands sent while the E22 transmits are lost (section 3.4). Ground
   station should transmit right after it receives a telemetry line. Not yet changed in
   `gs_pcb_v2`, which forwards host lines immediately.
2. **Acks only over MQTT.** No LoRa or XBee ack path. If needed, the ack queue could also be
   written to XBee.
3. **Telecommand split across IDLE gaps** becomes two lines. Send each command in one write.
4. **`STATE`, `PERSIST`, `SD` need ARM** even for their read-only forms, because the block is on
   the first word.
5. **`AT+QMTPUBEX` payload limit** for 1.4 KB image lines on the EC200U is not confirmed from
   the datasheet. Verify one full image over MQTT on the bench.
6. **`+QGPSLOC=2` field order** (`time,lat,lon,hdop,alt,fix,cog,spkm,spkn,date,nsat`) is from the
   Quectel GNSS manual; confirm on the bench that `SATS` and `LAT/LON` look right.
7. **No checksum verification** of image lines on the RF board; the ground station must verify `*hh`.
8. **Firmware not yet built in STM32CubeIDE by the author of this handoff**; host checks pass
   (section 9). Build, flash, and check all four UARTs with a logic analyser before flight.

## 9. Tests

Host self-checks (MinGW `gcc`), from `Tests/`:

```
gcc -I../Core/Inc test_telemetry.c ../Core/Src/telemetry.c -o t && ./t
gcc -Istubs -I../Core/Inc test_img_ring.c ../Core/Src/telemetry.c -o t && ./t
gcc -Istubs -I../Core/Inc test_telecommand.c ../Core/Src/telemetry.c -o t && ./t
```

They cover frame parse/format, the image ring and IMGACK, telecommand filtering and ARM, the
GYRO CAL / SD CLEAR hold, EC200U GNSS parsing and override, and SMS command extraction.

Bench sequence for a new board: flash, connect the FC (or replay a telemetry log at 921600 on
PC5), watch LoRa and XBee output on a USB-UART at 115200, subscribe to `cansat/011/#`, send
`BUZZ ON` on each source and check for `OK BUZZ ON` on the ack topic, then `ARM` and `DEPLOY`
to see `ARMED` and `OK DEPLOY`.
