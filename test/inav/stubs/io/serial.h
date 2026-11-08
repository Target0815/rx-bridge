#pragma once
#include <stdint.h>
typedef enum { FUNCTION_RX_SERIAL = 1 << 6 } serialPortFunction_e;
typedef enum { MODE_RX = 1, MODE_TX = 2, MODE_RXTX = 3 } portMode_t;
typedef enum {
    SERIAL_NOT_INVERTED = 0, SERIAL_INVERTED = 1 << 0, SERIAL_STOPBITS_1 = 0, SERIAL_STOPBITS_2 = 1 << 1,
    SERIAL_PARITY_NO = 0, SERIAL_PARITY_EVEN = 1 << 2, SERIAL_UNIDIR = 0, SERIAL_BIDIR = 1 << 3,
} portOptions_t;
typedef void (*serialReceiveCallbackPtr)(uint16_t data, void *rxCallbackData);
typedef struct serialPort_s { int dummy; } serialPort_t;
typedef struct { int identifier; } serialPortConfig_t;
const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e function);
serialPort_t *openSerialPort(int identifier, serialPortFunction_e function, serialReceiveCallbackPtr rxCallback,
                             void *rxCallbackData, uint32_t baudrate, portMode_t mode, portOptions_t options);
void serialWrite(serialPort_t *instance, uint8_t ch);
void serialWriteBuf(serialPort_t *instance, const uint8_t *data, int count);
