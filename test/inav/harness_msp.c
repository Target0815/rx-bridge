/* Host stand-in for the INAV CRSF telemetry task around the ORIGINAL telemetry/msp_shared.c:
 * buffers MSP-over-CRSF requests like telemetry/crsf.c and frames the reply like
 * crsfSendMspResponse(). mspFcProcessCommand answers like fc/fc_msp.c for the two
 * identification commands (MSP_FC_VARIANT "INAV", MSP_FC_VERSION 8.0.1). */
#include <string.h>
#include "platform.h"
#include "common/streambuf.h"
#include "common/crc.h"
#include "msp/msp.h"
#include "telemetry/msp_shared.h"

static uint8_t req[96];
static int req_len;
static uint8_t req_origin;
static bool pending;
static uint8_t out[64];
static int out_len;
int h_msp_requests;

bool bufferCrsfMspFrame(uint8_t *frameStart, int frameLength) {
    if (frameLength > (int)sizeof req) return false;
    memcpy(req, frameStart, frameLength);
    req_len = frameLength;
    return true;
}
void crsfScheduleMspResponse(uint8_t originId) { req_origin = originId; pending = true; h_msp_requests++; }

mspResult_e mspFcProcessCommand(mspPacket_t *cmd, mspPacket_t *reply, mspPostProcessFnPtr *pp) {
    (void)pp;
    reply->cmd = cmd->cmd;
    switch (cmd->cmd) {
    case 2: sbufWriteData(&reply->buf, "INAV", 4); return MSP_RESULT_ACK;
    case 3: sbufWriteU8(&reply->buf, 8); sbufWriteU8(&reply->buf, 0); sbufWriteU8(&reply->buf, 1); return MSP_RESULT_ACK;
    default: return MSP_RESULT_ERROR;
    }
}

static void respond(uint8_t *payload, const uint8_t size) {
    out[0] = 0xC8;                       /* CRSF_TELEMETRY_SYNC_BYTE */
    out[1] = (uint8_t)(size + 4);        /* payload + CRSF_FRAME_LENGTH_EXT_TYPE_CRC */
    out[2] = 0x7B;                       /* CRSF_FRAMETYPE_MSP_RESP */
    out[3] = req_origin;
    out[4] = 0xC8;                       /* CRSF_ADDRESS_FLIGHT_CONTROLLER */
    memcpy(&out[5], payload, size);
    out[5 + size] = crc8_dvb_s2_update(0, &out[2], size + 3);
    out_len = size + 6;
}

/* returns the CRSF reply frame the INAV telemetry task would send, or 0 */
int h_msp_task(uint8_t *frame) {
    static bool init;
    if (!init) { initSharedMsp(); init = true; }        /* INAV does this when telemetry starts */
    if (!pending) return 0;
    pending = false;
    out_len = 0;
    if (handleMspFrame(req, req_len)) {
        while (sendMspReply(58, respond)) { }
    }
    memcpy(frame, out, out_len);
    return out_len;
}
