/* Test host-side untuk control/pid.c, memakai implementasi ASLI.
 * pid.c hanya bergantung ke math.h, jadi bisa dilink native di PC.
 */
#include "test_common.h"
#include "pid.h"

int main(void)
{
    /* --- 1. P murni: output = kp * error, di-clamp ke [-1,1] ----------- */
    {
        PID_t p;
        PID_Init(&p, 0.5f, 0.0f, 0.0f);
        PID_Result_t r = PID_Compute(&p, 10.0f, 4.0f, 0.005f);
        CHECK_NEAR(r.p_term, 3.0f, 1e-5);
        CHECK_NEAR(r.output, 1.0f, 1e-5);   /* 3.0 -> clamp */

        r = PID_Compute(&p, 1.0f, 0.0f, 0.005f);
        CHECK_NEAR(r.output, 0.5f, 1e-5);
    }

    /* --- 2. dt di luar rentang -> integral & derivative tidak di-update -- */
    {
        PID_t p;
        PID_Init(&p, 0.0f, 1.0f, 0.0f);
        /* dt terlalu besar: i_term harus tetap 0, bukan meledak */
        PID_Result_t r = PID_Compute(&p, 1.0f, 0.0f, 10.0f);
        CHECK_NEAR(r.i_term, 0.0f, 1e-6);
        CHECK_NEAR(r.d_term, 0.0f, 1e-6);
        /* dt terlalu kecil: sama */
        r = PID_Compute(&p, 1.0f, 0.0f, 1e-9f);
        CHECK_NEAR(r.i_term, 0.0f, 1e-6);
    }

    /* --- 3. Anti-windup: integral tidak melewati batas ----------------- */
    {
        PID_t p;
        PID_Init(&p, 0.0f, 1.0f, 0.0f);
        for (int i = 0; i < 10000; i++) {
            PID_Compute(&p, 100.0f, 0.0f, 0.01f);
        }
        PID_Result_t r = PID_Compute(&p, 100.0f, 0.0f, 0.01f);
        CHECK(r.i_term <= 1.0f + 1e-5f);
        CHECK(r.output <= 1.0f + 1e-5f);
    }

    /* --- 4. Derivative-on-measurement (default) ------------------------ */
    {
        PID_t p;
        PID_Init(&p, 0.0f, 0.0f, 1.0f);
        /* Sample pertama: belum ada referensi -> d_term 0, bukan lonjakan. */
        PID_Result_t r = PID_Compute(&p, 0.0f, 5.0f, 0.1f);
        CHECK_NEAR(r.d_term, 0.0f, 1e-6);
        /* Measurement naik 1.0 dalam 0.1 s -> d = -(1.0/0.1) = -10 */
        r = PID_Compute(&p, 0.0f, 6.0f, 0.1f);
        CHECK_NEAR(r.d_term, -10.0f, 1e-4);
        /* Measurement DIAM -> turunan 0, walau error besar dan konstan. */
        r = PID_Compute(&p, 0.0f, 6.0f, 0.1f);
        CHECK_NEAR(r.d_term, 0.0f, 1e-5);
        /* Setpoint melompat: derivative-on-measurement tidak boleh kick. */
        r = PID_Compute(&p, 1000.0f, 6.0f, 0.1f);
        CHECK_NEAR(r.d_term, 0.0f, 1e-5);
    }

    /* --- 5. REGRESI: derivative-on-error = d(error)/dt ------------------ *
     * Versi lama menghitung `error / dt_seconds`, bukan turunan error.
     * Gejalanya paling jelas pada error KONSTAN: turunannya 0, tapi versi
     * lama mengeluarkan error/dt (mis. 5/0.1 = 50, dan makin besar makin
     * kecil dt -- 200x lipat pada dt=5 ms yang dipakai loop kontrol). */
    {
        PID_t p;
        PID_Init(&p, 0.0f, 0.0f, 1.0f);
        PID_SetOptions(&p, false, 0.0f);   /* derivative-on-error */

        /* Sample pertama: tidak ada referensi -> 0, bukan error/dt. */
        PID_Result_t r = PID_Compute(&p, 5.0f, 0.0f, 0.1f);
        CHECK_NEAR(r.d_term, 0.0f, 1e-6);

        /* Error tetap 5.0 -> d(error)/dt = 0. Versi lama: 50.0 */
        r = PID_Compute(&p, 5.0f, 0.0f, 0.1f);
        CHECK_NEAR(r.d_term, 0.0f, 1e-5);

        /* Error naik dari 5.0 ke 6.0 dalam 0.1 s -> d = +10 */
        r = PID_Compute(&p, 6.0f, 0.0f, 0.1f);
        CHECK_NEAR(r.d_term, 10.0f, 1e-4);

        /* Error turun kembali ke 5.0 -> d = -10 */
        r = PID_Compute(&p, 5.0f, 0.0f, 0.1f);
        CHECK_NEAR(r.d_term, -10.0f, 1e-4);

        /* PID_Reset() harus ikut membuang referensi error, supaya sample
         * pertama setelah reset tidak menghasilkan lonjakan turunan. */
        PID_Reset(&p);
        r = PID_Compute(&p, 5.0f, 0.0f, 0.1f);
        CHECK_NEAR(r.d_term, 0.0f, 1e-6);
    }

    /* --- 6. Reset membuang integral ------------------------------------ */
    {
        PID_t p;
        PID_Init(&p, 0.0f, 1.0f, 0.0f);
        PID_Compute(&p, 1.0f, 0.0f, 0.1f);
        PID_Reset(&p);
        PID_Result_t r = PID_Compute(&p, 0.0f, 0.0f, 0.1f);
        CHECK_NEAR(r.i_term, 0.0f, 1e-6);
    }

    TEST_SUMMARY("pid");
}
