/**
 * @file    test_compass.c
 * @brief   Host test untuk fusion/compass.c -- kelas bug yang tertangkap
 *          di sini murni aljabar (urutan/tanda komposisi rotasi tilt-
 *          compensation), TIDAK butuh bangku fisik. Verifikasi bangku
 *          fisik yang sesungguhnya (Compass_RemapToBody(), axis mapping
 *          chip mag) tetap TIDAK bisa digantikan test ini -- lihat
 *          catatan di compass.c.
 *
 * Riwayat: ditambahkan setelah review menemukan Compass_ComputeHeadingDeg()
 * salah urutan rotasi saat roll DAN pitch nonzero bersamaan (error hingga
 * ~180 derajat) -- kombinasi yang lolos dari test_ahrs_fusion.c karena
 * file itu sengaja tidak menguji jalur mag sama sekali. Test ini
 * dirancang supaya kelas bug yang sama otomatis tertangkap di masa
 * depan tanpa perlu bangku.
 */
#include "test_common.h"
#include <string.h>
#include "compass.h"

#define D2R (M_PI / 180.0)

/* Elementary rotation matrices (active rotation), independen dari
 * compass.c -- dipakai di sini hanya untuk MEMBANGUN sample sintetis
 * lewat forward transform level->body, supaya test ini tidak diam-diam
 * memakai formula yang sama dengan implementasi yang sedang diuji. */
static void RotX(double th, const double v[3], double out[3])
{
    out[0] = v[0];
    out[1] = v[1] * cos(th) - v[2] * sin(th);
    out[2] = v[1] * sin(th) + v[2] * cos(th);
}
static void RotY(double th, const double v[3], double out[3])
{
    out[0] = v[0] * cos(th) + v[2] * sin(th);
    out[1] = v[1];
    out[2] = -v[0] * sin(th) + v[2] * cos(th);
}

/* Forward transform level->body, konsisten dengan konvensi Euler
 * aerospace 321 (yaw-pitch-roll) yang implisit dipakai roll=atan2(ay,az)/
 * pitch=atan2(-ax,...) di ahrs_fusion.c: dari level, PITCH diterapkan
 * dulu (sekitar sumbu Y level), BARU ROLL (sekitar sumbu X hasil-pitch).
 * body = Rx(-roll) * Ry(-pitch) * level. */
static void LevelToBody(double roll_rad, double pitch_rad,
                        const double level[3], double body[3])
{
    double tmp[3];
    RotY(-pitch_rad, level, tmp);
    RotX(-roll_rad, tmp, body);
}

/* Bangun sample mag body-frame sintetis dari heading/roll/pitch yang
 * DIKETAHUI, lewat transform terbalik (inverse) dari yang dipakai
 * Compass_ComputeHeadingDeg() -- BUKAN salinan formula yang sama,
 * supaya test ini benar-benar independen dari implementasi. */
static void SyntheticMagSample(double heading_deg, double roll_deg, double pitch_deg,
                               double horiz, double vert, double out_body[3])
{
    double heading = heading_deg * D2R;
    /* level frame: X=depan(forward), Y=kiri(left), Z=atas(up). Medan
     * horizontal menunjuk ke utara (bearing 0, CW) -- proyeksi ke depan
     * = H*cos(heading), ke kiri = H*sin(heading) (lihat derivasi di
     * compass.c). Komponen vertikal (dip) independen dari heading. */
    double level[3] = { horiz * cos(heading), horiz * sin(heading), vert };
    LevelToBody(roll_deg * D2R, pitch_deg * D2R, level, out_body);
}

int main(void)
{
    /* --- 1. REGRESI UTAMA: tilt gabungan roll+pitch nonzero ------------ *
     * Ini persis kelas kasus yang gagal di formula lama (error hingga
     * ~180 derajat begitu roll DAN pitch nonzero bersamaan). Grid
     * heading x roll x pitch, plus medan dengan komponen vertikal
     * nonzero (dip) supaya mag_z ikut berperan penuh dalam kompensasi. */
    {
        const double headings[] = { 0.0, 37.0, 90.0, 123.0, 180.0, 234.0, 270.0, 311.0, 359.0 };
        const double rolls[]    = { -45.0, -20.0, 0.0, 20.0, 45.0 };
        const double pitches[]  = { -30.0, -10.0, 0.0, 10.0, 30.0 };
        double max_err = 0.0;

        for (size_t hi = 0; hi < sizeof(headings)/sizeof(headings[0]); hi++) {
            for (size_t ri = 0; ri < sizeof(rolls)/sizeof(rolls[0]); ri++) {
                for (size_t pi = 0; pi < sizeof(pitches)/sizeof(pitches[0]); pi++) {
                    double body[3];
                    SyntheticMagSample(headings[hi], rolls[ri], pitches[pi],
                                       0.5, 0.85, body);

                    float got = Compass_ComputeHeadingDeg((float)body[0], (float)body[1], (float)body[2],
                                                           (float)rolls[ri], (float)pitches[pi]);
                    double err = fabs(ang_diff((double)got, headings[hi]));
                    if (err > max_err) max_err = err;
                }
            }
        }
        printf("  max heading error (grid roll x pitch x heading): %.4f deg\n", max_err);
        CHECK(max_err < 0.05);   /* toleransi float, bukan nol persis */
    }

    /* --- 2. Kasus tunggal gampang dibaca (dokumentasi kontrak) --------- */
    {
        double body[3];

        SyntheticMagSample(0.0, 0.0, 0.0, 1.0, 0.0, body);
        CHECK_NEAR(Compass_ComputeHeadingDeg((float)body[0], (float)body[1], (float)body[2], 0.0f, 0.0f), 0.0, 0.01);

        SyntheticMagSample(90.0, 0.0, 0.0, 1.0, 0.0, body);
        CHECK_NEAR(Compass_ComputeHeadingDeg((float)body[0], (float)body[1], (float)body[2], 0.0f, 0.0f), 90.0, 0.01);

        /* Roll 90 derajat, tetap menghadap timur -- kasus ekstrem satu
         * sumbu (harusnya sudah benar bahkan di formula lama, tapi tetap
         * berguna sebagai sanity check dasar). */
        SyntheticMagSample(90.0, 90.0, 0.0, 1.0, 0.0, body);
        CHECK_NEAR(ang_diff(Compass_ComputeHeadingDeg((float)body[0], (float)body[1], (float)body[2], 90.0f, 0.0f), 90.0), 0.0, 0.05);

        /* Kasus yang GAGAL di formula lama: roll DAN pitch nonzero
         * bersamaan, menghadap barat daya. */
        SyntheticMagSample(225.0, 30.0, -20.0, 0.5, 0.85, body);
        CHECK_NEAR(ang_diff(Compass_ComputeHeadingDeg((float)body[0], (float)body[1], (float)body[2], 30.0f, -20.0f), 225.0), 0.0, 0.05);
    }

    /* --- 3. Output selalu di [0,360) ------------------------------------ */
    {
        double body[3];
        SyntheticMagSample(-40.0, 15.0, -15.0, 0.5, 0.85, body); /* heading negatif sengaja */
        float h = Compass_ComputeHeadingDeg((float)body[0], (float)body[1], (float)body[2], 15.0f, -15.0f);
        CHECK(h >= 0.0f && h < 360.0f);
    }

    /* --- 4. Kalibrasi: no-op (nol) sebelum valid, benar sesudahnya ------ */
    {
        CompassCalib_t calib;
        memset(&calib, 0, sizeof(calib));
        CHECK(!Compass_HasCalibration(&calib));

        float x, y, z;
        Compass_ApplyCalibration(&calib, 500, -300, 200, &x, &y, &z);
        CHECK_NEAR(x, 0.0, 1e-6);
        CHECK_NEAR(y, 0.0, 1e-6);
        CHECK_NEAR(z, 0.0, 1e-6);

        MagCalibResult_t result;
        memset(&result, 0, sizeof(result));
        result.offset_x = 100; result.offset_y = -50; result.offset_z = 10;
        result.scale_x = 1.0f; result.scale_y = 1.2f; result.scale_z = 0.9f;
        Compass_SetCalibration(&calib, &result);
        CHECK(Compass_HasCalibration(&calib));

        Compass_ApplyCalibration(&calib, 600, -50, 10, &x, &y, &z);
        CHECK_NEAR(x, (600.0 - 100.0) * 1.0, 1e-3);
        CHECK_NEAR(y, (-50.0 - (-50.0)) * 1.2, 1e-3);
        CHECK_NEAR(z, (10.0 - 10.0) * 0.9, 1e-3);
    }

    /* --- 5. Registrasi setting tidak crash (lihat catatan di compass.c) - */
    {
        CompassCalib_t calib;
        memset(&calib, 0, sizeof(calib));
        Compass_RegisterSettings(&calib);   /* cukup jangan crash */
    }

    /* --- 6. Pipeline penuh: Compass_RemapToBody() (mounting QMC5883) --- *
     * Beda dari test #1 (yang langsung memakai body-frame, tidak lewat
     * remap sama sekali), blok ini menguji Compass_RemapToBody() lewat
     * pipeline utuh: bangun medan magnet BODY-frame dari heading/roll/
     * pitch yang diketahui (SyntheticMagSample -- sama seperti #1), lalu
     * proyeksikan secara FISIK independen ke sumbu chip QMC5883 sesuai
     * mounting yang sudah diverifikasi bangku (lihat compass.c):
     *
     *   +X chip -> +Y body,  +Y chip -> -X body,  +Z chip -> +Z body
     *
     * Pembacaan chip = komponen medan body-frame di sepanjang tiap sumbu
     * chip (dot product), BUKAN memanggil balik Compass_RemapToBody()
     * dengan tanda dibalik -- kalau caranya begitu, test ini tidak akan
     * pernah gagal walau remap-nya salah (cuma menguji fungsi melawan
     * dirinya sendiri, lihat catatan di prompt task). Proyeksi:
     *
     *   cx_chip = dot(body, chip_X_dir) = dot(body, ( 0,1,0)) =  by_true
     *   cy_chip = dot(body, chip_Y_dir) = dot(body, (-1,0,0)) = -bx_true
     *   cz_chip = dot(body, chip_Z_dir) = dot(body, ( 0,0,1)) =  bz_true
     *
     * Barulah hasil "pembacaan chip" ini dilewatkan ke
     * Compass_RemapToBody() (fungsi yang diuji) lalu
     * Compass_ComputeHeadingDeg(), dan dicek kembali ke heading asli. */
    {
        const double headings[] = { 0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0, 359.0 };
        const double rolls[]    = { -30.0, 0.0, 30.0 };
        const double pitches[]  = { -20.0, 0.0, 20.0 };
        double max_err = 0.0;

        for (size_t hi = 0; hi < sizeof(headings)/sizeof(headings[0]); hi++) {
            for (size_t ri = 0; ri < sizeof(rolls)/sizeof(rolls[0]); ri++) {
                for (size_t pi = 0; pi < sizeof(pitches)/sizeof(pitches[0]); pi++) {
                    double body_true[3];
                    SyntheticMagSample(headings[hi], rolls[ri], pitches[pi],
                                       0.5, 0.85, body_true);

                    /* Proyeksi fisik independen ke sumbu chip (mounting
                     * QMC5883), bukan memanggil Compass_RemapToBody(). */
                    double chip_x =  body_true[1];
                    double chip_y = -body_true[0];
                    double chip_z =  body_true[2];

                    float bx, by, bz;
                    Compass_RemapToBody((float)chip_x, (float)chip_y, (float)chip_z,
                                        &bx, &by, &bz);
                    float got = Compass_ComputeHeadingDeg(bx, by, bz,
                                                          (float)rolls[ri], (float)pitches[pi]);
                    double err = fabs(ang_diff((double)got, headings[hi]));
                    if (err > max_err) max_err = err;
                }
            }
        }
        printf("  max heading error (Compass_RemapToBody pipeline): %.4f deg\n", max_err);
        CHECK(max_err < 0.05);

        /* Kasus tunggal gampang dibaca (dokumentasi kontrak mounting):
         * chip membaca medan murni di sepanjang sumbu chip +X sendiri
         * (chip_x=1, chip_y=0, chip_z=0), datar (roll=pitch=0). Lewat
         * remap: bx=-cy=0, by=cx=1, bz=cz=0 -- medan jadi murni di +Y
         * body (kiri pesawat). Dengan roll=pitch=0, Compass_ComputeHeadingDeg()
         * mereduksi jadi atan2(by,bx) = atan2(1,0) = 90 deg (nose timur --
         * field yang selalu menunjuk utara memang jatuh persis di sumbu
         * kiri body saat nose menghadap timur). Dihitung dari kode
         * (lihat pesan CHECK_NEAR di atas kalau meleset), bukan
         * diasumsikan satu arah dulu baru dicocokkan mundur. */
        {
            float bx, by, bz;
            Compass_RemapToBody(1.0f, 0.0f, 0.0f, &bx, &by, &bz);
            CHECK_NEAR(bx, 0.0, 1e-6);
            CHECK_NEAR(by, 1.0, 1e-6);
            CHECK_NEAR(bz, 0.0, 1e-6);
            float h = Compass_ComputeHeadingDeg(bx, by, bz, 0.0f, 0.0f);
            CHECK_NEAR(ang_diff((double)h, 90.0), 0.0, 0.05);
        }
    }

    TEST_SUMMARY("compass");
}
