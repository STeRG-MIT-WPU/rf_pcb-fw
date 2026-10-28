# rf_pcb-fw

Firmware for the CanSat RF board: STM32L4R5VIT6 (LQFP100), STM32CubeIDE / CubeMX project
(`rf_board.ioc`, FW_L4 V1.18.2), FreeRTOS (CMSIS-RTOS v2).

The board sits between the flight computer (FC) and three radios. It parses the FC telemetry
stream, re-transmits it on each radio at that radio's rate, relays camera image lines, and
forwards telecommands from the ground station back to the FC.

```
            USART3 921600                 UART5 115200   E22-900T22D LoRa   every 3 s (2.4 kbps air, half duplex)
 FC  <------------------------>  RF board <------------> 
   telemetry 10 Hz, $IMG lines            USART2 115200  XBee Pro           5 Hz + image lines
   <- telecommands                        UART4  921600  EC200U GSM         MQTT 15 Hz + image lines + SMS / 60 s
```

## Pin assignment

| Pin  | Function     | Direction | Connected to            | Notes                                   |
|------|--------------|-----------|-------------------------|-----------------------------------------|
| PC4  | USART3_TX    | out       | FC RX                   | telecommands, `IMGACK` to the FC        |
| PC5  | USART3_RX    | in        | FC TX                   | telemetry + `$IMG` lines, DMA1 Ch1 circular |
| PA0  | UART4_TX     | out       | EC200U RXD              | GSM / LTE modem                         |
| PA1  | UART4_RX     | in        | EC200U TXD              |                                         |
| PA2  | USART2_TX    | out       | XBee DIN                |                                         |
| PA3  | USART2_RX    | in        | XBee DOUT               |                                         |
| PA4  | GPIO input   | in        | XBee ASSOC              | association indicator, not used by firmware |
| PA5  | GPIO input   | in        | XBee nCTS               | high = XBee buffer full, TX waits up to 50 ms |
| PA6  | GPIO output  | out       | XBee nRTS               | driven low: always ready to receive     |
| PC12 | UART5_TX     | out       | E22 RXD                 | LoRa, transparent mode (M0/M1 tied on PCB) |
| PD2  | UART5_RX     | in        | E22 TXD                 |                                         |
| PH0  | RCC_OSC_IN   |           | 16 MHz crystal          | HSE, SYSCLK 120 MHz via PLL             |
| PH1  | RCC_OSC_OUT  |           | 16 MHz crystal          |                                         |
| PA13 | SWDIO        |           | debug header            |                                         |
| PA14 | SWCLK        |           | debug header            |                                         |

All UARTs are 8 data bits, no parity, 1 stop bit, no hardware flow control (XBee flow
control is done in software on PA5/PA6).

## UART baud rates

| UART   | Peripheral        | Baud    | RX method                          |
|--------|-------------------|---------|------------------------------------|
| USART3 | Flight computer   | 921600  | circular DMA + IDLE line           |
| UART4  | EC200U GSM        | 921600  | interrupt, IDLE line, 128 B buffer |
| USART2 | XBee Pro          | 115200  | interrupt, IDLE line               |
| UART5  | E22-900T22D LoRa  | 115200  | interrupt, IDLE line               |

The FC must already be configured for 921600 and the EC200U for 921600; the firmware does not change
module baud rates.

## Tasks (`Core/Src/freertos.c`)

| Task       | Priority     | Stack | Job |
|------------|--------------|-------|-----|
| `fc_comms` | AboveNormal  | 4 KB  | parse FC lines into `FC_tlm`, queue `$IMG` lines, send telecommands and `IMGACK` to the FC |
| `lora`     | Normal       | 4 KB  | telemetry every 3 s (240-byte chunk, 1 s gap, rest; ~1.2 s on air at 2.4 kbps, then listens for telecommands) |
| `xbee`     | Normal       | 4 KB  | telemetry at 5 Hz, one image line in the spare time of each 200 ms slot (~5 KB/s) |
| `gsm`      | BelowNormal  | 6 KB  | MQTT publish at 15 Hz, one image line per period, SMS with GNSS + mission time every 60 s, `+QMTRECV` and SMS (`+CMT`) telecommands, EC200U GNSS poll every 1 s |

Tunables (periods, APN, broker, client id, topics, SMS number, ring sizes) are the `#define`s
at the top of `freertos.c`. The APN/broker/topic/phone values are placeholders and must be set
before flight.

## Data formats

**FC telemetry line** (USART3, 10 Hz), parsed by `Core/Src/telemetry.c` into `FC_tlm`:

```
$,TEAM_ID,TIME,PKT,ACC(x|y|z),GYR(x|y|z),MAG(x|y|z),EUL(r|p|y),ALT,PRES,TEMP,VEL(x|y|z),
GNSS_TIME,LAT,LON,GNSS_ALT,SATS,GNSS_VEL(x|y),CURRENT,VOLT,BATT%,STATE,EFLAG,PROP_DEPLOY,RW_STATUS,#\r\n
```

Each radio sends the same line layout with its own packet counter (`LoRa_tlm`, `Xbee_tlm`,
`gsm_tlm` are `FC_tlm` with `packet_count` replaced).

**GNSS source:** the EC200U's own GNSS is switched on (`AT+QGPS=1`) and polled once a second
(`AT+QGPSLOC=2`). Whenever it reports more satellites than the FC's L89, its time, latitude,
longitude, altitude, satellite count and velocity replace the FC values in every outgoing
packet (LoRa, XBee, MQTT, SMS). A tie or no modem fix keeps the FC values.

**Image lines** from the FC are relayed verbatim over XBee and on the MQTT image topic:

```
$IMGSTART,<id>,<size>,<total>*hh
$IMG,<id>,<seq>,<total>,<len>,<base64>*hh
$IMGEND,<id>,<total>,<size>*hh
```

Each `$IMG` line carries 1024 raw bytes (the FC's `IMG LINK XBEE` default, ~1400 characters per
line). The 192-line relay ring holds ~190 KB of image. `IMG LINK MQTT` (10240 raw bytes, ~13.8 KB
lines) is not supported and the RF board refuses to forward that command.

`IMGACK,<id>,OK` is sent to the FC once both XBee and GSM have drained the image;
`IMGACK,<id>,FAIL` if a line was too long or the relay ring overflowed (the FC then resends).

**Telecommands** received on LoRa, XBee, the MQTT command topic or by SMS are cleaned up and
forwarded to the FC one line at a time, following the FC's `TELECOMMAND_HANDOFF.md`:

- Accepted input: raw FC console words (`BUZZ ON`) or `CMD,<team>,<verb>,<arg,...>`, which is
  translated to `<VERB> <ARG ...>`.
- The line is upper-cased, reduced to printable ASCII, spaces are collapsed, CRLF is appended.
  Lines starting with `$` or empty after cleanup are dropped.
- At most one command every 20 ms on USART3, lines up to 118 characters (FC limit 127).
- `ARM` is handled on the RF board and opens a 60 s window. Until then `RESET`, `BURN`, `PAYLOAD`,
  `DEPLOY`, `PARA`, `STATE`, `PERSIST`, `SD`, `BENCH`, `DIAG` and `SIM` lines are blocked.
- SMS: the modem delivers incoming texts directly (`AT+CNMI=2,2`); the first line of a text from
  `SMS_CMD_FROM` (defaults to `SMS_NUMBER`) is treated as a command, texts from other numbers are
  dropped. Acks for SMS commands still go to the MQTT ack topic.
- After `GYRO CAL` the RF board holds further commands for 13 s and after `SD CLEAR` for 3 s,
  because the FC's command task is blocked meanwhile. Queued commands go out afterwards in order.
- The FC does not reply on USART3. The RF board publishes one ack per line on the MQTT ack topic:
  `OK <line>` when forwarded, `ARMED`, `ARM? <line>` when blocked, `BAD <line>` when unusable.
  Confirm the effect through the `STATE`/`EFLAG` telemetry fields.

## Build and test

Open the folder in STM32CubeIDE and build. The linker gets `-u _printf_float` (set in
`.cproject`) because the formatter uses `%f` with newlib-nano.

Host self-checks (not part of the firmware build):

```
cd Tests
gcc -I../Core/Inc test_telemetry.c ../Core/Src/telemetry.c -o t && ./t
gcc -Istubs -I../Core/Inc test_img_ring.c ../Core/Src/telemetry.c -o t && ./t
gcc -Istubs -I../Core/Inc test_telecommand.c ../Core/Src/telemetry.c -o t && ./t
```
