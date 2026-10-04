/* Test host-side untuk calibration/calib_gate.c, memakai implementasi ASLI.
 *
 * calib_gate.c sengaja dibuat tanpa dependensi hardware justru supaya
 * kebijakan gerbang arming bisa diuji seluruhnya di PC seperti ini --
 * satu-satunya bagian dari rantai arming yang tidak butuh board.
 *
 * Yang diuji: SELURUH 16 kombinasi dari 4 input boolean, bukan sekadar
 * beberapa kasus yang "terlihat penting". Gerbang ini kecil, jadi tabel
 * kebenaran penuh itu murah -- dan bug aslinya (arming ditolak permanen
 * kalau MPU6050 tidak terpasang) justru ada di kombinasi yang gampang
 * dianggap tidak penting.
 */
#include "test_common.h"
#include "calib_gate.h"

/* Kebijakan yang seharusnya, ditulis ulang sebagai tabel literal --
 * SENGAJA bukan sebagai ekspresi boolean. Kalau ekspektasinya ditulis
 * sebagai rumus, test cuma menyalin bug yang sama dari implementasinya.
 *
 * Kolom: p_present, p_calib, s_present, s_calib, harapan
 * Aturan: primer WAJIB hadir & terkalibrasi; sekunder wajib terkalibrasi
 *         HANYA jika hadir.
 */
static const struct {
    bool p_present, p_calib, s_present, s_calib, expect;
    const char *why;
} k_table[16] = {
    /* primer tidak hadir -> selalu tolak, apa pun kondisi sekunder */
    { false, false, false, false, false, "tidak ada IMU sama sekali" },
    { false, false, false, true,  false, "primer absen" },
    { false, false, true,  false, false, "primer absen" },
    { false, false, true,  true,  false, "primer absen, sekunder siap" },
    { false, true,  false, false, false, "primer absen (calib basi)" },
    { false, true,  false, true,  false, "primer absen (calib basi)" },
    { false, true,  true,  false, false, "primer absen (calib basi)" },
    { false, true,  true,  true,  false, "primer absen (calib basi)" },

    /* primer hadir tapi belum terkalibrasi -> selalu tolak */
    { true,  false, false, false, false, "primer belum terkalibrasi" },
    { true,  false, false, true,  false, "primer belum terkalibrasi" },
    { true,  false, true,  false, false, "keduanya belum terkalibrasi" },
    { true,  false, true,  true,  false, "primer belum terkalibrasi" },

    /* primer hadir & terkalibrasi -> hanya sekunder YANG HADIR yang menahan */
    { true,  true,  false, false, true,  "REGRESI: MPU6050 dicabut harus tetap bisa arm" },
    { true,  true,  false, true,  true,  "sekunder absen, flag calib-nya tidak relevan" },
    { true,  true,  true,  false, false, "sekunder hadir tapi belum terkalibrasi" },
    { true,  true,  true,  true,  true,  "semua siap" },
};

int main(void)
{
    printf("== calib_gate: tabel kebenaran 16 kombinasi ==\n");

    for (unsigned i = 0; i < 16u; i++) {
        bool got = CalibGate_ArmAllowed(k_table[i].p_present, k_table[i].p_calib,
                                        k_table[i].s_present, k_table[i].s_calib);
        if (got != k_table[i].expect) {
            printf("  kasus %u (%d%d%d%d): %s\n", i,
                   k_table[i].p_present, k_table[i].p_calib,
                   k_table[i].s_present, k_table[i].s_calib, k_table[i].why);
        }
        CHECK(got == k_table[i].expect);
    }

    /* Pastikan tabel di atas benar-benar mencakup 16 kombinasi unik, bukan
     * ada baris yang tidak sengaja terduplikasi saat diedit. */
    unsigned seen = 0u;
    for (unsigned i = 0; i < 16u; i++) {
        unsigned key = ((unsigned)k_table[i].p_present << 3)
                     | ((unsigned)k_table[i].p_calib   << 2)
                     | ((unsigned)k_table[i].s_present << 1)
                     | ((unsigned)k_table[i].s_calib);
        seen |= (1u << key);
    }
    CHECK(seen == 0xFFFFu);

    /* Regresi eksplisit atas bug yang memicu seluruh perubahan ini:
     * `primary.calibrated && secondary.calibrated` menolak arming permanen
     * kalau IMU sekunder tidak terpasang. */
    CHECK(CalibGate_ArmAllowed(true, true, false, false) == true);

    TEST_SUMMARY("calib_gate");
}
