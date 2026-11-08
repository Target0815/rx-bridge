#pragma once

#define FW_NAME    "rx-bridge"
#define FW_VERSION "0.3.2"

/* GPIO assignment, named from the point of view of the module (a UART is wired crossed:
 * TX of the module to RX of the FC, RX of the module to TX of the FC).
 * GPIO26-28 exist on Raspberry Pi Pico / Pico 2, Waveshare RP2040-Zero / RP2350-Zero and
 * Seeed XIAO RP2040 / RP2350 (D0-D2). Override from CMake with -DPIN_LINE=... etc. */
#ifndef PIN_LINE
#define PIN_LINE   26   /* receiver S.Port / F.Port / FBUS wire */
#endif
#ifndef PIN_RX
#define PIN_RX     27   /* module input  -> connect to TX of the flight controller */
#endif
#ifndef PIN_TX
#define PIN_TX     28   /* module output -> connect to RX of the flight controller */
#endif

/* defaults until settings are saved with the 's' command */
#define DEFAULT_INVERTED 1      /* FrSky receivers: inverted */
#define DEFAULT_HOLD_US  10     /* transparent mode: LINE actively held for 10 us after the FC stops */
#define DEFAULT_MODE     0      /* 0 = transparent inverter, 1 = converter F.Port/FBUS -> CRSF */
#define HOLD_US_MIN      1
#define HOLD_US_MAX      100

#define CRSF_BAUD        420000
#define FBUS_BAUD        460800
#define FPORT_BAUD       115200
#define CONV_WATCHDOG_MS 300
