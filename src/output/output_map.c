/**
 * @file    output_map.c
 * @brief   Implementasi output mapping. Lihat output_map.h untuk
 *          dokumentasi API dan kontrak dependency ke dshot.h/pwm_servo.h.
 */

#include "output_map.h"
#include "command_handler.h"
#include <stddef.h>
#include <string.h>

/* --- Dependency dari output/dshot.c dan output/pwm_servo.c (belum dibuat,
 * lihat kontrak di output_map.h) --- */
extern void DShot_SetThrottle(uint8_t motor_index, float throttle_0_1);
extern void PwmServo_SetPosition(uint8_t servo_index, float position_minus1_1);

static inline float output_map_clampf(float value, float min, float max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

void OutputMap_InitDefault(OutputMap_t *map)
{
    if (map == NULL) {
        return;
    }

    map->slot_count = 0;

    /* Urutan sengaja mengikuti MixerChannel_t supaya index array
     * slots[] == nilai enum MixerChannel_t, jadi OutputMap_WriteFromMixer()
     * bisa lookup langsung tanpa scan linear. Kalau urutan ini diubah,
     * OutputMap_WriteFromMixer() WAJIB ikut disesuaikan (lihat komentar
     * di sana). */
    map->slots[map->slot_count++] = (OutputMapSlot_t){
        .mixer_channel = MIXER_CH_MOTOR0, .role = OUTPUT_ROLE_MOTOR, .driver_index = 0
    };
    map->slots[map->slot_count++] = (OutputMapSlot_t){
        .mixer_channel = MIXER_CH_AIL_L, .role = OUTPUT_ROLE_SERVO, .driver_index = 0
    };
    map->slots[map->slot_count++] = (OutputMapSlot_t){
        .mixer_channel = MIXER_CH_AIL_R, .role = OUTPUT_ROLE_SERVO, .driver_index = 1
    };
    map->slots[map->slot_count++] = (OutputMapSlot_t){
        .mixer_channel = MIXER_CH_VTAIL_L, .role = OUTPUT_ROLE_SERVO, .driver_index = 2
    };
    map->slots[map->slot_count++] = (OutputMapSlot_t){
        .mixer_channel = MIXER_CH_VTAIL_R, .role = OUTPUT_ROLE_SERVO, .driver_index = 3
    };
}

bool OutputMap_SetSlot(OutputMap_t *map, MixerChannel_t mixer_channel,
                        OutputRole_t role, uint8_t driver_index)
{
    if (map == NULL || mixer_channel >= MIXER_CH_COUNT) {
        return false;
    }

    /* Cari slot yang sudah memetakan channel ini -> update in-place. */
    for (uint8_t i = 0; i < map->slot_count; i++) {
        if (map->slots[i].mixer_channel == mixer_channel) {
            map->slots[i].role = role;
            map->slots[i].driver_index = driver_index;
            return true;
        }
    }

    /* Belum ada -> tambah slot baru kalau masih muat. */
    if (map->slot_count >= OUTPUT_MAP_MAX_SLOTS) {
        return false;
    }

    map->slots[map->slot_count].mixer_channel = mixer_channel;
    map->slots[map->slot_count].role = role;
    map->slots[map->slot_count].driver_index = driver_index;
    map->slot_count++;

    return true;
}

void OutputMap_WriteFromMixer(const OutputMap_t *map, const MixerOutput_t *mixer_output,
                               bool armed)
{
    if (map == NULL || mixer_output == NULL) {
        return;
    }

    for (uint8_t i = 0; i < map->slot_count; i++) {
        const OutputMapSlot_t *slot = &map->slots[i];

        if (slot->mixer_channel >= MIXER_CH_COUNT) {
            continue; /* entri korup/tidak valid, lewati defensif */
        }

        float value = mixer_output->channels[slot->mixer_channel];

        switch (slot->role) {
            case OUTPUT_ROLE_MOTOR: {
                /* Armed-state guard WAJIB — lihat dokumentasi di
                 * output_map.h. Ini satu-satunya tempat nilai motor final
                 * dipaksa idle saat disarmed, supaya guard tidak perlu
                 * diduplikasi di mixer.c maupun dshot.c. */
                float throttle = armed ? output_map_clampf(value, 0.0f, 1.0f) : 0.0f;
                DShot_SetThrottle(slot->driver_index, throttle);
                break;
            }
            case OUTPUT_ROLE_SERVO: {
                float position = output_map_clampf(value, -1.0f, 1.0f);
                PwmServo_SetPosition(slot->driver_index, position);
                break;
            }
            case OUTPUT_ROLE_NONE:
            default:
                /* Slot sengaja tidak dipakai — tidak ada driver dipanggil. */
                break;
        }
    }
}

const OutputMapSlot_t *OutputMap_GetSlotByIndex(const OutputMap_t *map, uint8_t index)
{
    if (map == NULL || index >= map->slot_count) {
        return NULL;
    }
    return &map->slots[index];
}

/* ---------------------------------------------------------------------------
 * CMD_MOTOR_TEST / CMD_SERVO_TEST (Comms #2)
 *
 * Wire format (dari src/core/commands/actuatorTestCommands.ts di
 * faas-configurator, sudah jadi kode yang berjalan di sisi web):
 *   Request:  [outputIndex: u8][value: f32 little-endian]
 *   Response: [status: u8]   (0 = ok, 1 = gagal -- index/role tidak
 *              cocok, ATAU payload_len kurang dari yang diharapkan; TIDAK
 *              pernah berarti "ditolak karena armed" di sini, karena
 *              kasus armed sudah dicegat dispatch() lebih dulu lewat
 *              CMD_ERROR/ERROR_ARMED_REJECTED sebelum handler ini
 *              sempat dipanggil -- lihat OutputMap_RegisterCommands()
 *              di output_map.h).
 *
 * `outputIndex` merujuk ke index logis output.pin0..pinN (urutan
 * pendaftaran OutputMap_GetSlotByIndex(), BUKAN driver_index internal)
 * -- untuk default V-tail: 0=motor, 1-4=servo AIL_L/AIL_R/VTAIL_L/VTAIL_R.
 * ------------------------------------------------------------------------- */
static OutputMap_t *s_protocol_output_map;

static void handle_actuator_test(const protocol_frame_t *frame, OutputRole_t expected_role)
{
    uint8_t status = 1; /* default: gagal/ditolak, override kalau sukses */

    if (frame->payload_len >= 5U) {
        uint8_t output_index = frame->payload[0];
        float value;
        memcpy(&value, &frame->payload[1], sizeof(float));

        /* NaN (f32 dari wire) lolos dari output_map_clampf() karena semua
         * perbandingan dengan NaN false -- tanpa cek ini, NaN bisa sampai
         * ke DShot_SetThrottle()/PwmServo_SetPosition(). Diperlakukan
         * sama seperti request tidak valid lain: status=1. (value != value
         * hanya true untuk NaN; +-Inf aman, sudah ke-clamp.) */
        const OutputMapSlot_t *slot = OutputMap_GetSlotByIndex(s_protocol_output_map, output_index);
        if (value == value && slot != NULL && slot->role == expected_role) {
            if (expected_role == OUTPUT_ROLE_MOTOR) {
                value = output_map_clampf(value, 0.0f, 1.0f);
                DShot_SetThrottle(slot->driver_index, value);
            } else {
                value = output_map_clampf(value, -1.0f, 1.0f);
                PwmServo_SetPosition(slot->driver_index, value);
            }
            status = 0;
        }
    }

    Protocol_SendFrame(expected_role == OUTPUT_ROLE_MOTOR ? CMD_MOTOR_TEST : CMD_SERVO_TEST,
                        frame->request_id, &status, sizeof(status));
}

static void handle_motor_test(const protocol_frame_t *frame)
{
    handle_actuator_test(frame, OUTPUT_ROLE_MOTOR);
}

static void handle_servo_test(const protocol_frame_t *frame)
{
    handle_actuator_test(frame, OUTPUT_ROLE_SERVO);
}

void OutputMap_RegisterCommands(OutputMap_t *map)
{
    s_protocol_output_map = map;
    CommandHandler_Register(CMD_MOTOR_TEST, handle_motor_test);
    CommandHandler_Register(CMD_SERVO_TEST, handle_servo_test);
}
