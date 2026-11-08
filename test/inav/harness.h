#pragma once
#include <stdint.h>
#include "io/serial.h"
#include "telemetry/smartport.h"
extern uint32_t h_now_us;
extern uint8_t  h_tx[512];
extern int      h_tx_len;
extern int      h_lq;
extern smartPortPayload_t h_next_payload;
extern serialReceiveCallbackPtr h_rx_cb;
