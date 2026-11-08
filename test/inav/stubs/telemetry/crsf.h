#pragma once
#include <stdint.h>
#include <stdbool.h>
#define CRSF_MSP_RX_BUF_SIZE 512
#define CRSF_MSP_TX_BUF_SIZE 512
void crsfScheduleMspResponse(uint8_t requestOriginID);
bool bufferCrsfMspFrame(uint8_t *frameStart, int frameLength);
