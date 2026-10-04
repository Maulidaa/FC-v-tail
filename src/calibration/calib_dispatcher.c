/**
 * calib_dispatcher.c
 *
 * Lihat calib_dispatcher.h untuk kontrak publik & catatan asumsi nilai
 * calib_type.
 */

#include "calib_dispatcher.h"
#include "calib_accel_gyro.h"
#include "calib_mag.h"
#include "command_handler.h"
#include "error.h"
#include <stddef.h>

/* "Belum pernah start kalibrasi apa pun sejak boot" — dibedakan dari
 * CALIB_TYPE_ACCEL_GYRO/CALIB_TYPE_MAG supaya GetStatus() bisa
 * membedakan "IDLE karena belum pernah dipakai" vs "IDLE karena baru
 * saja di-stop". */
#define NO_ACTIVE_TYPE (0xFFu)

static uint8_t s_last_type = NO_ACTIVE_TYPE;

static bool is_type_running(CalibType_t type)
{
    switch (type) {
        case CALIB_TYPE_ACCEL_GYRO:
            return CalibAccelGyro_GetState() == CALIB_AG_STATE_IN_PROGRESS;
        case CALIB_TYPE_MAG:
            return CalibMag_GetState() == CALIB_MAG_STATE_IN_PROGRESS;
        default:
            return false;
    }
}

bool CalibDispatcher_Start(CalibType_t type)
{
    /* Tolak kalau ada jenis LAIN yang sedang berjalan. Restart jenis
     * yang sama diperbolehkan (fallthrough ke Start() masing-masing
     * modul, yang me-reset progress dari awal). */
    CalibType_t other = (type == CALIB_TYPE_ACCEL_GYRO)
                         ? CALIB_TYPE_MAG : CALIB_TYPE_ACCEL_GYRO;
    if (is_type_running(other)) {
        return false;
    }

    switch (type) {
        case CALIB_TYPE_ACCEL_GYRO:
            CalibAccelGyro_Start();
            break;
        case CALIB_TYPE_MAG:
            CalibMag_Start();
            break;
        default:
            return false;
    }

    s_last_type = (uint8_t)type;
    return true;
}

void CalibDispatcher_Stop(CalibType_t type)
{
    switch (type) {
        case CALIB_TYPE_ACCEL_GYRO:
            CalibAccelGyro_Stop();
            break;
        case CALIB_TYPE_MAG:
            CalibMag_Stop();
            break;
        default:
            break;
    }
    /* s_last_type sengaja tidak direset ke NO_ACTIVE_TYPE di sini —
     * GetStatus() setelah Stop() harus tetap melaporkan state IDLE
     * dari jenis yang baru dihentikan, bukan "belum pernah dipakai". */
}

void CalibDispatcher_FeedImuSample(int16_t ax, int16_t ay, int16_t az,
                                    int16_t gx, int16_t gy, int16_t gz)
{
    /* CalibAccelGyro_FeedSample() sudah no-op internal kalau state
     * bukan IN_PROGRESS, jadi dispatcher tidak perlu cek dobel di sini. */
    CalibAccelGyro_FeedSample(ax, ay, az, gx, gy, gz);
}

void CalibDispatcher_FeedMagSample(int16_t x, int16_t y, int16_t z)
{
    CalibMag_FeedSample(x, y, z);
}

bool CalibDispatcher_GetStatus(uint8_t *out_type, uint8_t *out_state,
                                uint8_t *out_progress)
{
    if (out_type == NULL || out_state == NULL || out_progress == NULL) {
        return false;
    }
    if (s_last_type == NO_ACTIVE_TYPE) {
        return false;
    }

    *out_type = s_last_type;

    if ((CalibType_t)s_last_type == CALIB_TYPE_ACCEL_GYRO) {
        *out_state    = (uint8_t)CalibAccelGyro_GetState();
        *out_progress = CalibAccelGyro_GetProgressPercent();
    } else {
        *out_state    = (uint8_t)CalibMag_GetState();
        *out_progress = CalibMag_GetProgressPercent();
    }

    return true;
}

/* ---------------------------------------------------------------------------
 * Handler protokol (Comms #3) -- CMD_CALIB_*
 *
 * Wire format (protocol.md Bagian 10):
 *   START/STOP : request payload kosong. Respons sukses = frame kosong
 *                (command_id sama, request_id di-echo) sebagai ack.
 *   STATUS     : respons [calib_type:u8][state:u8][progress_pct:u8],
 *                state 0=idle 1=in_progress 2=done 3=failed -- nilai enum
 *                CalibAccelGyroState_t/CalibMagState_t sudah identik
 *                dengan wire, jadi GetStatus() tinggal diserialisasi.
 * Ditolak dengan CMD_ERROR/ERROR_BUSY (sudah ada di error.h, maknanya
 * memang "kalibrasi lain sedang berjalan") kalau START bentrok.
 *
 * TIDAK armed-gated (sesuai protocol.md; s_armed_gated_commands[]
 * sengaja tidak disentuh). TEMUAN untuk tim, bukan diputuskan di sini:
 * START kalibrasi accel/gyro/mag saat armed=true secara teknis
 * diterima -- untuk mag tidak berbahaya, tapi kalibrasi accel/gyro
 * yang mengubah bias saat terbang jelas tidak diinginkan. Kalau tim
 * setuju, cukup tambahkan CMD_CALIB_ACCEL_GYRO_START ke daftar gated
 * (dan update protocol.md), bukan diam-diam dari sini.
 * ------------------------------------------------------------------------- */
static void send_calib_ack(const protocol_frame_t *frame, uint16_t command_id)
{
    Protocol_SendFrame(command_id, frame->request_id, NULL, 0);
}

static void handle_calib_start(const protocol_frame_t *frame, CalibType_t type)
{
    if (!CalibDispatcher_Start(type)) {
        Error_Send(frame->request_id, ERROR_BUSY, frame->command_id);
        return;
    }
    send_calib_ack(frame, frame->command_id);
}

static void handle_calib_mag_start(const protocol_frame_t *frame)
{
    handle_calib_start(frame, CALIB_TYPE_MAG);
}

static void handle_calib_mag_stop(const protocol_frame_t *frame)
{
    CalibDispatcher_Stop(CALIB_TYPE_MAG);
    send_calib_ack(frame, CMD_CALIB_MAG_STOP);
}

/* Guard konflik dengan task_calib_recover() (main.c): modul
 * calib_accel_gyro.c itu SINGLETON dan task pemulihan memakainya
 * langsung (tanpa lewat dispatcher ini). Start manual saat pemulihan
 * aktif akan me-reset sesi yang sedang berjalan atau sebaliknya.
 * Di sini HANYA ditolak (ERROR_BUSY) -- siapa yang seharusnya mengalah
 * (auto-recovery vs operator) adalah keputusan desain terbuka, lihat
 * laporan Comms #3, BUKAN diputuskan sepihak di kode ini.
 * Stop manual sengaja TIDAK di-guard tapi hanya berdampak kalau sesi
 * dispatcher (bukan recovery) yang aktif -- lihat handle_..._stop. */
static void handle_calib_accel_gyro_start(const protocol_frame_t *frame)
{
    if (CalibRecover_IsActive()) {
        Error_Send(frame->request_id, ERROR_BUSY, frame->command_id);
        return;
    }
    handle_calib_start(frame, CALIB_TYPE_ACCEL_GYRO);
}

static void handle_calib_accel_gyro_stop(const protocol_frame_t *frame)
{
    /* CalibDispatcher_Stop(ACCEL_GYRO) memanggil CalibAccelGyro_Stop()
     * yang meng-IDLE-kan singleton yang SAMA dengan yang dipakai
     * recovery -- kalau recovery sedang RUNNING, Stop manual akan
     * mematikan sample-nya di tengah sesi (state di main.c tetap
     * RUNNING sampai timeout, lalu BACKOFF/ulang). Jadi guard yang
     * sama dengan START dipakai di sini juga. */
    if (CalibRecover_IsActive()) {
        Error_Send(frame->request_id, ERROR_BUSY, frame->command_id);
        return;
    }
    CalibDispatcher_Stop(CALIB_TYPE_ACCEL_GYRO);
    send_calib_ack(frame, CMD_CALIB_ACCEL_GYRO_STOP);
}

static void handle_calib_status(const protocol_frame_t *frame)
{
    uint8_t payload[3] = { 0u, 0u, 0u };
    uint8_t type, state, progress;

    /* false = belum pernah ada kalibrasi di-start sejak boot -- kondisi
     * normal, dibalas IDLE (calib_type 0), bukan CMD_ERROR. */
    if (CalibDispatcher_GetStatus(&type, &state, &progress)) {
        payload[0] = type;
        payload[1] = state;
        payload[2] = progress;
    }
    Protocol_SendFrame(CMD_CALIB_STATUS, frame->request_id, payload, sizeof(payload));
}

void CalibDispatcher_RegisterCommands(void)
{
    CommandHandler_Register(CMD_CALIB_ACCEL_GYRO_START, handle_calib_accel_gyro_start);
    CommandHandler_Register(CMD_CALIB_ACCEL_GYRO_STOP,  handle_calib_accel_gyro_stop);
    CommandHandler_Register(CMD_CALIB_MAG_START,        handle_calib_mag_start);
    CommandHandler_Register(CMD_CALIB_MAG_STOP,         handle_calib_mag_stop);
    CommandHandler_Register(CMD_CALIB_STATUS,           handle_calib_status);
}
