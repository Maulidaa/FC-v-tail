#include "error.h"
#include "protocol.h"

/* ============================================================================
 * error.c
 * Lihat error.h untuk catatan penting soal status "proposal, belum
 * dikonfirmasi tim" pada bentuk payload CMD_ERROR ini.
 * ============================================================================ */

int Error_Send(uint8_t request_id, error_code_t code, uint16_t original_command_id)
{
    uint8_t payload[3];
    payload[0] = (uint8_t)code;
    payload[1] = (uint8_t)(original_command_id & 0xFFu);
    payload[2] = (uint8_t)((original_command_id >> 8) & 0xFFu);

    return Protocol_SendFrame(CMD_ERROR, request_id, payload, sizeof(payload));
}
