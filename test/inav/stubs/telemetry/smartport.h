#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "io/serial.h"
#define SMARTPORT_MSP_TX_BUF_SIZE 256
#define SMARTPORT_MSP_RX_BUF_SIZE 64
typedef struct smartPortPayload_s {
    uint8_t  frameId;
    uint16_t valueId;
    uint32_t data;
} __attribute__((packed)) smartPortPayload_t;
typedef void smartPortWriteFrameFn(const smartPortPayload_t *payload);
bool initSmartPortTelemetryExternal(smartPortWriteFrameFn *smartPortWriteFrameExternal);
void processSmartPortTelemetry(smartPortPayload_t *payload, volatile bool *hasRequest, const uint32_t *requestTimeout);
void smartPortSendByte(uint8_t c, uint16_t *checksum, serialPort_t *port);
bool smartPortPayloadContainsMSP(const smartPortPayload_t *payload);
void smartPortWriteFrameSerial(const smartPortPayload_t *payload, serialPort_t *port, uint16_t checksum);
