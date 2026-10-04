/**
 * @file    output_map.h
 * @brief   Layer pemetaan channel logis (dari mixer.c) -> role fisik
 *          (MOTOR/SERVO) + driver mana yang menulis nilainya. Lihat
 *          firmware-architecture-stm32f411.md Bagian 3.8 dan Bagian 1
 *          (revisi V-tail, 5 channel aktif).
 *
 * Alur data lengkap:
 *   stabilize.c (pid.c) -> mixer.c -> OUTPUT_MAP (file ini) -> dshot.c / pwm_servo.c
 *
 * Kenapa perlu layer terpisah, bukan mixer.c langsung panggil dshot.c/pwm_servo.c:
 * - Supaya pemetaan slot fisik -> role bisa diubah lewat setting (mis. user
 *   sengaja pasang motor di slot lain, atau kelak dukung airframe lain
 *   dengan jumlah channel beda) TANPA mixer.c perlu tahu apa-apa soal
 *   DShot/PWM.
 * - Supaya armed-state guard untuk channel MOTOR bisa dipusatkan di satu
 *   tempat (lihat OutputMap_WriteFromMixer()), konsisten dengan catatan
 *   revisi arsitektur: "perluasan cakupan armed-state guard ke
 *   output-mapping".
 *
 * Selain jalur mixer di atas, file ini juga mendaftarkan handler
 * CMD_MOTOR_TEST/CMD_SERVO_TEST (Comms #2) lewat
 * OutputMap_RegisterCommands() -- jalur BERBEDA dari
 * OutputMap_WriteFromMixer() (dipicu command test manual dari web, bukan
 * dari mixer.c tiap siklus kontrol), tapi tetap lewat layer ini karena
 * sama-sama butuh terjemahan index/role -> driver_index + driver yang
 * sama (dshot.c/pwm_servo.c).
 *
 * === Dependency ke dshot.h / pwm_servo.h (belum dibuat) ===
 * File ini meng-extern fungsi berikut, diasumsikan disediakan modul
 * `output/dshot.c/h` dan `output/pwm_servo.c/h` (dikerjakan berikutnya):
 *
 *   void DShot_SetThrottle(uint8_t motor_index, float throttle_0_1);
 *   void PwmServo_SetPosition(uint8_t servo_index, float position_minus1_1);
 *
 * motor_index/servo_index di sini SUDAH terpisah per jenis driver (index 0
 * untuk motor pertama, index 0 untuk servo pertama, dst) — bukan index
 * channel logis global — karena DShot jalan di TIM1 sementara servo di
 * TIM3/TIM4, dua peripheral & driver berbeda.
 */

#ifndef OUTPUT_MAP_H
#define OUTPUT_MAP_H

#include <stdbool.h>
#include <stdint.h>
#include "mixer.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OUTPUT_MAP_MAX_SLOTS (MIXER_CH_COUNT)

typedef enum {
    OUTPUT_ROLE_NONE = 0, /* slot tidak dipakai — nilai mixer untuk channel ini diabaikan */
    OUTPUT_ROLE_MOTOR,    /* ditulis lewat DShot_SetThrottle() */
    OUTPUT_ROLE_SERVO,    /* ditulis lewat PwmServo_SetPosition() */
} OutputRole_t;

/**
 * @brief Satu entri pemetaan: channel logis dari mixer (MixerChannel_t)
 *        -> role fisik + index di dalam driver role tersebut.
 */
typedef struct {
    MixerChannel_t mixer_channel;
    OutputRole_t role;
    uint8_t driver_index; /* index di dalam DShot atau PwmServo, terpisah per role */
} OutputMapSlot_t;

typedef struct {
    OutputMapSlot_t slots[OUTPUT_MAP_MAX_SLOTS];
    uint8_t slot_count;
} OutputMap_t;

/**
 * @brief Isi map dengan pemetaan default sesuai airframe v1 (V-tail):
 *   MIXER_CH_MOTOR0  -> OUTPUT_ROLE_MOTOR, driver_index 0 (PA8/TIM1_CH1, DShot)
 *   MIXER_CH_AIL_L   -> OUTPUT_ROLE_SERVO, driver_index 0 (PB0/TIM3_CH3)
 *   MIXER_CH_AIL_R   -> OUTPUT_ROLE_SERVO, driver_index 1 (PB1/TIM3_CH4)
 *   MIXER_CH_VTAIL_L -> OUTPUT_ROLE_SERVO, driver_index 2 (PB8/TIM4_CH3)
 *   MIXER_CH_VTAIL_R -> OUTPUT_ROLE_SERVO, driver_index 3 (PB9/TIM4_CH4)
 *
 * Pemetaan driver_index servo (0-3) ini konvensi internal output_map.c,
 * BUKAN dari mixer.h — pwm_servo.c yang menentukan servo index N ada di
 * pin fisik mana (lihat dependency note di pwm_servo.h nanti).
 */
void OutputMap_InitDefault(OutputMap_t *map);

/**
 * @brief Ubah role & driver_index untuk channel mixer tertentu. Dipakai
 *        kalau nanti ada command setting untuk remap output (mis. dari
 *        web configurator), atau airframe lain dengan jumlah channel beda.
 *
 * @return false kalau mixer_channel tidak valid atau map penuh.
 */
bool OutputMap_SetSlot(OutputMap_t *map, MixerChannel_t mixer_channel,
                        OutputRole_t role, uint8_t driver_index);

/**
 * @brief Cari slot berdasar index logis URUTAN PENDAFTARAN
 *        (0..slot_count-1, sesuai urutan OutputMap_InitDefault()/
 *        OutputMap_SetSlot() menambah entri) -- BUKAN berdasar
 *        MixerChannel_t atau driver_index. Ini index yang sama dengan
 *        `output.pin0..pinN` di grup setting `output_mapping`
 *        (protocol.md Bagian 8), dipakai handler CMD_MOTOR_TEST/
 *        CMD_SERVO_TEST (lihat OutputMap_RegisterCommands()) untuk
 *        menerjemahkan `outputIndex` dari web ke slot fisik yang benar.
 * @return NULL kalau index di luar jangkauan slot_count saat ini (atau
 *         `map` NULL) -- caller cukup cek NULL, tidak ada kondisi lain
 *         yang perlu ditangani terpisah.
 */
const OutputMapSlot_t *OutputMap_GetSlotByIndex(const OutputMap_t *map, uint8_t index);

/**
 * @brief Daftarkan handler CMD_MOTOR_TEST/CMD_SERVO_TEST ke
 *        command_handler.c, terikat ke instance `map` yang diberikan.
 *        Pola sama persis dengan Nav_RegisterProtocolHandlers() ->
 *        s_protocol_nav (lihat navigation.c): pointer disimpan di
 *        variabel static file-local, dipakai handler tanpa perlu
 *        parameter tambahan di command_handler_fn_t.
 *
 *        WAJIB dipanggil SETELAH CommandHandler_Init() (yang me-reset
 *        tabel registrasi command_handler.c sebelum mendaftarkan
 *        CMD_GET_STATUS bawaannya sendiri -- registrasi sebelum itu
 *        akan ikut terhapus). Boleh dipanggil kapan pun relatif
 *        terhadap OutputMap_InitDefault(map) -- OutputMap_GetSlotByIndex()
 *        aman dipanggil sebelum map terisi (slot_count masih 0 dari
 *        static-init, jadi selalu return NULL, bukan crash), asal map
 *        SUDAH terisi sebelum command pertama benar-benar diterima
 *        dari web (bukan sebelum registrasi ini).
 *
 *        Guard armed TIDAK diulang di dalam handler -- CMD_MOTOR_TEST
 *        dan CMD_SERVO_TEST sudah masuk s_armed_gated_commands[]
 *        terpusat di command_handler.c; dispatch() menolaknya (lewat
 *        CMD_ERROR/ERROR_ARMED_REJECTED) SEBELUM handler di sini pernah
 *        dipanggil kalau armed=true. Lihat catatan kebijakan gating
 *        terpusat di command_handler.h.
 *
 * @param map instance OutputMap_t yang dipakai untuk melayani kedua
 *        command ini (di firmware saat ini: &s_fc.stabilize.output_map,
 *        lihat main.c). Boleh NULL (handler akan selalu balas status=1
 *        tanpa crash), tapi seharusnya selalu diisi instance nyata.
 */
void OutputMap_RegisterCommands(OutputMap_t *map);

/**
 * @brief Tulis seluruh channel dari MixerOutput_t ke driver fisik yang
 *        sesuai (DShot_SetThrottle / PwmServo_SetPosition), sesuai
 *        pemetaan di `map`.
 *
 * Armed-state guard (WAJIB, bukan opsional): untuk slot ber-role
 * OUTPUT_ROLE_MOTOR, kalau `armed` == false, nilai yang benar-benar
 * dikirim ke DShot_SetThrottle() DIPAKSA 0.0f (idle) terlepas dari nilai
 * di `mixer_output` — motor tidak boleh berputar dalam kondisi apa pun
 * saat disarmed. Servo TIDAK di-gate di sini (servo tetap boleh gerak saat
 * disarmed, mis. untuk lihat respons kontrol permukaan sebelum terbang);
 * guard khusus untuk CMD_SERVO_TEST ada di command_handler.c
 * (armed_state.c), bukan di layer ini.
 *
 * @param map           Pemetaan channel -> role+driver_index saat ini.
 * @param mixer_output   Hasil Mixer_Compute().
 * @param armed          State armed terkini (dari armed_state.c).
 */
void OutputMap_WriteFromMixer(const OutputMap_t *map, const MixerOutput_t *mixer_output,
                               bool armed);

#ifdef __cplusplus
}
#endif

#endif /* OUTPUT_MAP_H */
