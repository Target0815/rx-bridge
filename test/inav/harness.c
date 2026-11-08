/* Host stubs around the original INAV receiver parsers (rx/crsf.c, rx/fport.c, rx/fport2.c). */
#include <string.h>
#include "platform.h"
#include "drivers/time.h"
#include "io/serial.h"
#include "rx/rx.h"
#include "telemetry/smartport.h"
#include "harness.h"

uint32_t h_now_us;
uint8_t  h_tx[512];
int      h_tx_len;
int      h_lq;
smartPortPayload_t h_next_payload;
serialReceiveCallbackPtr h_rx_cb;
rxLinkStatistics_t rxLinkStatistics;

static serialPort_t port;
static const serialPortConfig_t cfg = { 0 };
static smartPortWriteFrameFn *ext_writer;

timeUs_t micros(void) { return h_now_us; }
timeUs_t microsISR(void) { return h_now_us; }
timeMs_t millis(void) { return h_now_us / 1000; }

const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e f) { (void)f; return &cfg; }
serialPort_t *openSerialPort(int id, serialPortFunction_e f, serialReceiveCallbackPtr cb, void *d, uint32_t baud,
                             portMode_t mode, portOptions_t opt) {
    (void)id; (void)f; (void)d; (void)baud; (void)mode; (void)opt;
    h_rx_cb = cb;
    return &port;
}
void serialWrite(serialPort_t *p, uint8_t ch) { (void)p; if (h_tx_len < (int)sizeof h_tx) h_tx[h_tx_len++] = ch; }
void serialWriteBuf(serialPort_t *p, const uint8_t *d, int n) { for (int i = 0; i < n; i++) serialWrite(p, d[i]); }
void lqTrackerSet(void *lq, int v) { (void)lq; h_lq = v; }

bool initSmartPortTelemetryExternal(smartPortWriteFrameFn *w) { ext_writer = w; return true; }
void processSmartPortTelemetry(smartPortPayload_t *payload, volatile bool *clearToSend, const uint32_t *t) {
    (void)payload; (void)t;
    if (*clearToSend) { ext_writer(&h_next_payload); *clearToSend = false; }
}
bool smartPortPayloadContainsMSP(const smartPortPayload_t *p) { (void)p; return false; }

/* the two helpers below follow INAV telemetry/smartport.c exactly */
#define FSSP_DLE 0x7D
#define FSSP_DLE_XOR 0x20
#define FSSP_START_STOP 0x7E
void frskyCheckSumStep(uint16_t *checksum, uint8_t byte);
void frskyCheckSumFini(uint16_t *checksum);
void smartPortSendByte(uint8_t c, uint16_t *checksum, serialPort_t *p) {
    if (c == FSSP_DLE || c == FSSP_START_STOP) {
        serialWrite(p, FSSP_DLE);
        serialWrite(p, c ^ FSSP_DLE_XOR);
    } else {
        serialWrite(p, c);
    }
    if (checksum != NULL) frskyCheckSumStep(checksum, c);
}
void smartPortWriteFrameSerial(const smartPortPayload_t *payload, serialPort_t *p, uint16_t checksum) {
    uint8_t *data = (uint8_t *)payload;
    for (unsigned i = 0; i < sizeof(smartPortPayload_t); i++) smartPortSendByte(*data++, &checksum, p);
    frskyCheckSumFini(&checksum);
    smartPortSendByte(checksum, NULL, p);
}
