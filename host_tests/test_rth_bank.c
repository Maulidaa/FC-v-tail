/**
 * @file    test_rth_bank.c
 * @brief   Host test untuk rth_bank_from_heading() (src/main.c) -- fungsi
 *          `static`, tidak bisa di-link langsung karena main.c menyeret
 *          header MCU/CMSIS (masalah yang sama seperti alasan
 *          settings.h/settings_stub.c dipakai untuk test_compass, lihat
 *          Makefile). Fungsi disalin verbatim di bawah.
 *
 * Konteks bug yang ditutup fix ini: target_deg (bearing nav, searah
 * jarum jam / CW) dan current_deg (att->yaw_deg AHRS, berlawanan arah
 * jarum jam / CCW -- lihat ahrs_fusion.c: `mag_yaw_ccw_equiv =
 * -mag->heading_deg`) memakai konvensi putaran yang BERLAWANAN.
 * rth_bank_from_heading() versi lama membandingkan keduanya tanpa
 * konversi, sehingga error yang dihitung salah persis saat heading
 * sungguh-sungguh sejajar target (lihat kasus #2 di bawah).
 *
 * CATATAN PENTING soal contoh angka di prompt task Fase B: prompt
 * menyebut pasangan target_deg=90, current_deg=-270 sebagai "sudah
 * menghadap timur, persis target" (error diharapkan ~0). Sudah
 * diverifikasi ulang secara independen terhadap konvensi yang
 * SUNGGUH-SUNGGUH dipakai kode (ahrs_fusion.c, baris `mag_yaw_ccw_equiv
 * = -mag->heading_deg`) -- klaim itu TERBALIK. current_deg=-270 setara
 * (mod 360) dengan yaw CCW=90, yang lewat konversi -yaw berarti bearing
 * CW=270 (barat) -- 180 derajat dari target (timur), bukan sejajar.
 * Pasangan yang BENAR-BENAR sejajar dengan target=90 (timur) adalah
 * current_deg=270 (atau -90, kongruen mod 360) -- lihat
 * bearing_cw_to_yaw_ccw(90)=270 di bawah. Fungsi/fix di main.c TETAP
 * BENAR (konsisten dengan konvensi ahrs_fusion.c); yang keliru cuma
 * contoh angka di prompt. Kasus #2a menguji pasangan yang benar-benar
 * sejajar (270), kasus #2c menguji pasangan LITERAL dari prompt (-270)
 * dengan nilai harapan yang sudah dikoreksi (~180, bukan ~0) supaya
 * kekeliruan ini tidak diam-diam terulang di masa depan.
 */
#include "test_common.h"
#include <string.h>

/* --- Konstanta, disalin dari src/main.c -- WAJIB disinkronkan manual
 * kalau nilai aslinya berubah (lihat pembagian-tugas-firmware-4-orang.md:
 * fungsi ini bukan tanggung jawab task Fase B untuk mengubah nilai). */
#define RTH_MAX_BANK_DEG            25.0f
#define RTH_HEADING_TO_BANK_GAIN    0.80f

/* SALINAN PERSIS dari rth_bank_from_heading() di src/main.c (baris ~629,
 * versi SESUDAH fix Fase B). WAJIB disinkronkan manual kalau fungsi
 * aslinya berubah -- pola sama seperti host_tests/settings.h terhadap
 * src/settings/settings.h. */
static float rth_bank_from_heading(float target_deg, float current_deg)
{
    float current_cw_deg = -current_deg;

    float error = target_deg - current_cw_deg;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    float bank = error * RTH_HEADING_TO_BANK_GAIN;
    if (bank > RTH_MAX_BANK_DEG) return RTH_MAX_BANK_DEG;
    if (bank < -RTH_MAX_BANK_DEG) return -RTH_MAX_BANK_DEG;
    return bank;
}

/* Versi SEBELUM fix (bug CW/CCW asli) -- disalin persis dari kode lama,
 * dipakai HANYA di test ini sebagai pembanding regresi (TIDAK dipanggil
 * dari mana pun di firmware). Tujuannya membuktikan fix di atas benar-
 * benar mengubah hasil untuk kasus yang sama, bukan cuma menguji kode
 * baru secara terisolasi ("melawan dirinya sendiri"). */
static float rth_bank_from_heading_OLD_BUGGY(float target_deg, float current_deg)
{
    float error = target_deg - current_deg;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    float bank = error * RTH_HEADING_TO_BANK_GAIN;
    if (bank > RTH_MAX_BANK_DEG) return RTH_MAX_BANK_DEG;
    if (bank < -RTH_MAX_BANK_DEG) return -RTH_MAX_BANK_DEG;
    return bank;
}

/* Ground truth INDEPENDEN: konversi bearing CW (nav, seperti target_deg)
 * -> yaw CCW (seperti att->yaw_deg AHRS), memakai pola konversi yang
 * SUNGGUH dipakai kode (ahrs_fusion.c: mag_yaw_ccw_equiv =
 * -mag->heading_deg), TAPI diimplementasikan ulang di sini secara
 * terpisah -- dipakai untuk MEMBANGUN sample sintetis dari bearing yang
 * diketahui, bukan untuk menyalin rumus rth_bank_from_heading() yang
 * sedang diuji. */
static double bearing_cw_to_yaw_ccw(double bearing_cw_deg)
{
    double yaw = fmod(-bearing_cw_deg, 360.0);
    if (yaw < 0.0) yaw += 360.0;
    return yaw;
}

int main(void)
{
    /* --- 1. Kondisi awal sederhana (dari prompt): target=90, current=0 -
     * current_cw = -0 = 0, error = +90 (belok kanan/positif) -- sanity
     * check tanda dasar sebelum masuk kasus wrap yang lebih rumit. */
    {
        float bank = rth_bank_from_heading(90.0f, 0.0f);
        CHECK(bank > 0.0f);
        /* |error|=90 * gain 0.8 = 72, di atas RTH_MAX_BANK_DEG (25) ->
         * clamp ke +25 persis. */
        CHECK_NEAR(bank, RTH_MAX_BANK_DEG, 1e-4);
    }

    /* --- 2. Kasus inti: heading benar-benar sejajar target, tapi beda
     * representasi/kelipatan 360 -- kelas bug yang ditutup fix ini. */

    /* 2a. Pasangan yang BENAR-BENAR sejajar (target=90 timur, current
     * dari bearing_cw_to_yaw_ccw(90)=270, ground truth independen).
     * Fix baru harus dekat nol; formula lama pada PASANGAN YANG SAMA
     * harus terbukti gagal (mentok limit, arah salah) -- bukti langsung
     * fix ini mengubah hasil untuk kasus yang benar-benar penting. */
    {
        double yaw_ccw = bearing_cw_to_yaw_ccw(90.0);
        CHECK_NEAR(yaw_ccw, 270.0, 1e-9);

        float bank_new = rth_bank_from_heading(90.0f, (float)yaw_ccw);
        float bank_old = rth_bank_from_heading_OLD_BUGGY(90.0f, (float)yaw_ccw);

        CHECK_NEAR(bank_new, 0.0, 0.5);   /* fix: sudah sejajar, bank ~0 */
        CHECK(fabs(bank_old) > 15.0);     /* lama: dikira jauh melenceng, mentok limit */
    }

    /* 2b. Kongruen mod 360 dengan 2a (current=-90, bukan 270) -- fungsi
     * harus tetap benar untuk representasi yang belum di-wrap ke
     * [0,360), bukan cuma nilai yang sudah "rapi". */
    {
        float bank = rth_bank_from_heading(90.0f, -90.0f);
        CHECK_NEAR(bank, 0.0, 0.5);
    }

    /* 2c. Pasangan LITERAL dari prompt task (target=90, current=-270),
     * diuji dengan nilai harapan yang SUDAH DIKOREKSI (lihat catatan
     * panjang di kepala file): ini kasus berlawanan penuh (barat vs
     * target timur), error seharusnya mentok di limit clamp, BUKAN ~0
     * seperti klaim asli di prompt. */
    {
        float bank = rth_bank_from_heading(90.0f, -270.0f);
        CHECK_NEAR(fabs(bank), RTH_MAX_BANK_DEG, 1e-3);
    }

    /* --- 3. Wrap di 0/360 pada kedua sisi (bearing nav maupun yaw CCW) - */
    {
        /* true_bearing=358 (CW), target=2 (CW) -- pesawat perlu belok
         * +4 derajat (kanan tipis) untuk sampai ke target, BUKAN lompat
         * ~356/-356. */
        double yaw_ccw = bearing_cw_to_yaw_ccw(358.0);
        float bank = rth_bank_from_heading(2.0f, (float)yaw_ccw);
        CHECK(bank > 0.0f);
        CHECK(bank < RTH_MAX_BANK_DEG);
        CHECK_NEAR(bank, 4.0 * (double)RTH_HEADING_TO_BANK_GAIN, 0.05);

        /* true_bearing=1 (CW), target=359 (CW) -- pesawat perlu belok
         * -2 derajat (kiri tipis), representasi yaw CCW-nya sendiri
         * dekat wrap 360/0. */
        yaw_ccw = bearing_cw_to_yaw_ccw(1.0);
        bank = rth_bank_from_heading(359.0f, (float)yaw_ccw);
        CHECK(bank < 0.0f);
        CHECK(fabs(bank) < RTH_MAX_BANK_DEG);
        CHECK_NEAR(bank, -2.0 * (double)RTH_HEADING_TO_BANK_GAIN, 0.05);
    }

    /* --- 4. Sapuan acak: dibandingkan ke ground truth independen -------
     * true_bearing acak (posisi fisik sesungguhnya, CW) diubah ke yaw
     * CCW lewat bearing_cw_to_yaw_ccw() (independen dari rumus yang
     * diuji) -- ditambah kelipatan 360 acak (k) untuk menguji robustness
     * terhadap representasi yang belum di-wrap. Dibandingkan ke
     * ang_diff(target, true_bearing) dari test_common.h (utilitas
     * shortest-signed-diff independen, dipakai juga di test_compass.c),
     * bukan menyalin ulang wrap-loop implementasi. */
    {
        srand(12345u);
        int checked = 0;
        for (int i = 0; i < 500; i++) {
            double target = (double)(rand() % 3600) / 10.0;
            double true_bearing = (double)(rand() % 3600) / 10.0;
            int k = (rand() % 5) - 2;   /* -2..2 */
            double yaw_ccw = bearing_cw_to_yaw_ccw(true_bearing) + 360.0 * k;

            double expected_error = ang_diff(target, true_bearing);
            double expected_bank = expected_error * (double)RTH_HEADING_TO_BANK_GAIN;
            if (expected_bank > RTH_MAX_BANK_DEG)  expected_bank = RTH_MAX_BANK_DEG;
            if (expected_bank < -RTH_MAX_BANK_DEG) expected_bank = -RTH_MAX_BANK_DEG;

            float bank = rth_bank_from_heading((float)target, (float)yaw_ccw);
            CHECK_NEAR(bank, expected_bank, 0.1);
            checked++;
        }
        printf("  sapuan acak: %d pasangan target/current diverifikasi\n", checked);
    }

    TEST_SUMMARY("rth_bank");
}
