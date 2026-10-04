/* Test host-side untuk system/armed_state.c, memakai implementasi ASLI.
 *
 * armed_state.c memang didesain menerima snapshot ArmedStateInputs_t
 * (bukan meng-include header modul lain) justru supaya bisa dilink
 * sendirian di PC seperti ini -- lihat alasannya di armed_state.h.
 *
 * Fokus test:
 *   1. Baterai SUDAH TIDAK jadi syarat preflight (Tugas A). Struct-nya
 *      bahkan tidak lagi punya field battery_ok, jadi test ini juga
 *      berfungsi sebagai pengaman kompilasi: kalau field itu kembali,
 *      make_ok_inputs() di bawah tidak akan mengisinya dan gerbangnya
 *      diam-diam jadi non-deterministik.
 *   2. Urutan pelaporan kegagalan (run_preflight melaporkan kegagalan
 *      PERTAMA saja -- checklist OLED bergantung pada itu).
 *   3. calibration_done benar-benar menahan arming (invarian #1).
 *   4. Disarm saat switch arm turun.
 */
#include "test_common.h"
#include <string.h>
#include "armed_state.h"

/* Semua syarat lolos. Setiap test mengubah SATU field dari sini, supaya
 * jelas field mana yang sedang diuji dan tidak ada syarat yang diam-diam
 * ikut gagal. */
static ArmedStateInputs_t make_ok_inputs(void)
{
    ArmedStateInputs_t in;
    memset(&in, 0, sizeof(in));
    in.arm_switch_engaged = true;
    in.throttle_low       = true;
    in.rx_link_ok         = true;
    in.attitude_valid     = true;
    in.imu_disagreement   = false;
    in.calibration_done   = true;
    in.blackbox_ready     = true;
    return in;
}

/* Satu siklus dari kondisi bersih: Init() lalu Update() sekali. */
static bool try_arm(const ArmedStateInputs_t *in)
{
    ArmedState_Init();
    ArmedState_Update(in);
    return ArmedState_IsArmed();
}

int main(void)
{
    /* --- 1. Jalur bahagia --------------------------------------------- */
    {
        ArmedStateInputs_t in = make_ok_inputs();
        CHECK(try_arm(&in) == true);
        CHECK(ArmedState_GetLastPreflightError() == PREFLIGHT_OK);
        CHECK(ArmedState_Get() == ARMED_STATE_ARMED);
        CHECK(Armed_IsArmed() == 1);
    }

    /* --- 2. Tidak ada lagi gerbang baterai ----------------------------- */
    {
        /* Tidak ada input yang bisa memicu PREFLIGHT_LOW_BATTERY lagi:
         * struct inputnya tidak punya field baterai sama sekali. Yang bisa
         * dipastikan di sini adalah (a) arming lolos dengan input lengkap
         * di atas tanpa pernah menyebut baterai, dan (b) tidak ada jalur
         * yang mengembalikan PREFLIGHT_LOW_BATTERY. Untuk (b), sapu semua
         * kombinasi 6 input boolean dan pastikan kode itu tidak pernah
         * muncul. */
        int seen_low_battery = 0;
        for (unsigned m = 0; m < 64u; m++) {
            ArmedStateInputs_t in;
            memset(&in, 0, sizeof(in));
            in.arm_switch_engaged = true;
            in.throttle_low     = (m & 1u)  != 0u;
            in.rx_link_ok       = (m & 2u)  != 0u;
            in.attitude_valid   = (m & 4u)  != 0u;
            in.imu_disagreement = (m & 8u)  != 0u;
            in.calibration_done = (m & 16u) != 0u;
            in.blackbox_ready   = (m & 32u) != 0u;

            ArmedState_Init();
            ArmedState_Update(&in);
            if (ArmedState_GetLastPreflightError() == PREFLIGHT_LOW_BATTERY) {
                seen_low_battery++;
            }
        }
        CHECK(seen_low_battery == 0);

        /* Konstanta enumnya sendiri SENGAJA dipertahankan supaya nilai
         * numerik PREFLIGHT_BLACKBOX_FAULT tidak bergeser (web configurator
         * di luar repo ini membaca nilai numeriknya). */
        CHECK((int)PREFLIGHT_LOW_BATTERY     == 6);
        CHECK((int)PREFLIGHT_BLACKBOX_FAULT  == 7);
    }

    /* --- 3. Urutan penolakan ------------------------------------------- */
    {
        /* throttle dicek paling dulu: walau SEMUA syarat lain juga gagal,
         * yang dilaporkan harus throttle. */
        ArmedStateInputs_t in;
        memset(&in, 0, sizeof(in));
        in.arm_switch_engaged = true;   /* sisanya false / gagal semua */
        in.imu_disagreement   = true;
        CHECK(try_arm(&in) == false);
        CHECK(ArmedState_GetLastPreflightError() == PREFLIGHT_THROTTLE_NOT_LOW);

        in = make_ok_inputs(); in.rx_link_ok = false;
        CHECK(try_arm(&in) == false);
        CHECK(ArmedState_GetLastPreflightError() == PREFLIGHT_NO_RX_LINK);

        in = make_ok_inputs(); in.attitude_valid = false;
        CHECK(try_arm(&in) == false);
        CHECK(ArmedState_GetLastPreflightError() == PREFLIGHT_IMU_FAULT);

        in = make_ok_inputs(); in.imu_disagreement = true;
        CHECK(try_arm(&in) == false);
        CHECK(ArmedState_GetLastPreflightError() == PREFLIGHT_IMU_DISAGREE);

        in = make_ok_inputs(); in.blackbox_ready = false;
        CHECK(try_arm(&in) == false);
        CHECK(ArmedState_GetLastPreflightError() == PREFLIGHT_BLACKBOX_FAULT);
    }

    /* --- 4. calibration_done benar-benar menahan (invarian #1) ---------- */
    {
        ArmedStateInputs_t in = make_ok_inputs();
        in.calibration_done = false;
        CHECK(try_arm(&in) == false);
        CHECK(ArmedState_GetLastPreflightError() == PREFLIGHT_UNCALIBRATED);

        /* Tetap ditolak berapa kali pun switch arm ditahan -- tidak ada
         * "akhirnya lolos" karena dicoba berulang. */
        for (int i = 0; i < 100; i++) {
            ArmedState_Update(&in);
            CHECK(ArmedState_IsArmed() == false);
        }

        /* Begitu kalibrasi pulih (task_calib_recover di main.c), cycle
         * BERIKUTNYA langsung boleh arm tanpa reboot -- switch arm masih
         * ditahan, dan preflight memang dievaluasi ulang tiap cycle selama
         * state-nya masih DISARMED. */
        in.calibration_done = true;
        ArmedState_Update(&in);
        CHECK(ArmedState_IsArmed() == true);
    }

    /* --- 5. Disarm saat switch turun ------------------------------------ */
    {
        ArmedStateInputs_t in = make_ok_inputs();
        CHECK(try_arm(&in) == true);

        in.arm_switch_engaged = false;
        ArmedState_Update(&in);
        CHECK(ArmedState_IsArmed() == false);
        CHECK(ArmedState_Get() == ARMED_STATE_DISARMED);
    }

    /* --- 6. Perilaku yang TIDAK boleh berubah oleh perubahan ini -------- */
    {
        /* (a) Preflight hanya dievaluasi saat transisi: sudah ARMED lalu
         *     syarat memburuk -> TETAP armed (itu keputusan desain
         *     armed_state.h, bukan bug). */
        ArmedStateInputs_t in = make_ok_inputs();
        CHECK(try_arm(&in) == true);
        in.rx_link_ok       = false;
        in.attitude_valid   = false;
        in.calibration_done = false;
        in.throttle_low     = false;
        ArmedState_Update(&in);
        CHECK(ArmedState_IsArmed() == true);

        /* (b) ForceDisarm tetap satu-satunya jalan keluar paksa. */
        ArmedState_ForceDisarm();
        CHECK(ArmedState_IsArmed() == false);

        /* (c) Switch arm belum ditarik -> jangan timpa hasil preflight
         *     terakhir dengan apa pun. */
        ArmedState_Init();
        ArmedStateInputs_t idle = make_ok_inputs();
        idle.arm_switch_engaged = false;
        idle.throttle_low       = false;
        ArmedState_Update(&idle);
        CHECK(ArmedState_GetLastPreflightError() == PREFLIGHT_OK);
        CHECK(ArmedState_Get() == ARMED_STATE_DISARMED);

        /* (d) Pointer NULL tidak boleh mengubah state / crash. */
        ArmedState_Update(NULL);
        CHECK(ArmedState_IsArmed() == false);
    }

    TEST_SUMMARY("armed_state");
}
