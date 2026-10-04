/**
 * @file    compass.c
 * @brief   Implementasi compass.h. Lihat compass.h untuk alur pakai dan
 *          kontrak konvensi heading.
 */

#include "compass.h"
#include "settings.h"

#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------- */
/* Kalibrasi                                                            */
/* ------------------------------------------------------------------- */

void Compass_SetCalibration(CompassCalib_t *ctx, const MagCalibResult_t *result)
{
    if (ctx == NULL || result == NULL) {
        return;
    }
    ctx->offset_x = result->offset_x;
    ctx->offset_y = result->offset_y;
    ctx->offset_z = result->offset_z;
    ctx->scale_x  = result->scale_x;
    ctx->scale_y  = result->scale_y;
    ctx->scale_z  = result->scale_z;
    ctx->valid    = true;
}

bool Compass_HasCalibration(const CompassCalib_t *ctx)
{
    return (ctx != NULL) && ctx->valid;
}

void Compass_ApplyCalibration(const CompassCalib_t *ctx,
                              int16_t raw_x, int16_t raw_y, int16_t raw_z,
                              float *out_x, float *out_y, float *out_z)
{
    if (out_x == NULL || out_y == NULL || out_z == NULL) {
        return;
    }
    if (!Compass_HasCalibration(ctx)) {
        *out_x = 0.0f;
        *out_y = 0.0f;
        *out_z = 0.0f;
        return;
    }
    *out_x = ((float)raw_x - (float)ctx->offset_x) * ctx->scale_x;
    *out_y = ((float)raw_y - (float)ctx->offset_y) * ctx->scale_y;
    *out_z = ((float)raw_z - (float)ctx->offset_z) * ctx->scale_z;
}

/* ------------------------------------------------------------------- */
/* Remap axis -> body frame                                             */
/* ------------------------------------------------------------------- */

void Compass_RemapToBody(float cal_x, float cal_y, float cal_z,
                         float *out_bx, float *out_by, float *out_bz)
{
    if (out_bx == NULL || out_by == NULL || out_bz == NULL) {
        return;
    }
    /* Terverifikasi bangku (uji putar fisik): QMC5883 di board ini
     * terpasang dengan +Z sudah searah +Z body (atas), tapi X/Y
     * terputar 90 derajat pada bidang horizontal:
     *   +X chip -> menghadap KIRI     (= +Y body)
     *   +Y chip -> menghadap BELAKANG (= -X body)
     *   +Z chip -> menghadap ATAS     (= +Z body, tidak berubah)
     * Verifikasi handedness: chip_X x chip_Y = (0,1,0)x(-1,0,0) = (0,0,1)
     * = chip_Z -- rotasi murni, bukan pencerminan, jadi tidak perlu
     * balik tanda tambahan di luar tiga baris ini. Khusus QMC5883 --
     * kalau board memakai HMC5883, mounting belum tentu sama, uji
     * ulang terpisah sebelum dipakai untuk chip itu.
     *
     * CATATAN PENTING (ditemukan saat menulis fungsi ini, di luar
     * cakupan task remap axis): main.c (task_ahrs(), sekitar baris
     * 610) memanggil Compass_RemapToBody() TANPA cabang per-chip --
     * remap yang sama dipakai persis sama untuk kedua kemungkinan
     * s_active_mag_chip (MAG_CHIP_HMC5883 maupun MAG_CHIP_QMC5883,
     * lihat main.c). Task ini secara eksplisit "khusus QMC5883, jangan
     * sentuh main.c" -- jadi remap di bawah HANYA benar kalau board
     * yang dipakai memang QMC5883 (atau kalau HMC5883 kebetulan
     * mounting-nya identik, belum diverifikasi). Kalau firmware ini
     * dipakai dengan HMC5883 aktif, orientasi heading-nya BELUM
     * terverifikasi bangku sama sekali -- perlu task terpisah untuk
     * menambah percabangan chip di titik panggil (main.c) sebelum
     * mempercayai heading dari HMC5883. */
    *out_bx = -cal_y;
    *out_by =  cal_x;
    *out_bz =  cal_z;
}

/* ------------------------------------------------------------------- */
/* Heading tilt-compensated                                             */
/* ------------------------------------------------------------------- */

#define DEG_TO_RAD_F   (0.017453292519943295f)
#define RAD_TO_DEG_F   (57.29577951308232f)

static float wrap_deg_0_360(float deg)
{
    if (!(deg == deg)) {   /* NaN */
        return 0.0f;
    }
    deg = fmodf(deg, 360.0f);
    if (deg < 0.0f) {
        deg += 360.0f;
    }
    return deg;
}

/*
 * Derivasi (lihat compass.h untuk kontrak frame/konvensi):
 *
 * Body frame di sini adalah X depan, Y kiri, Z atas (right-handed,
 * X x Y = Z), SAMA konvensi tanda dengan roll/pitch AHRS
 * (ahrs_fusion.c): roll positif = sayap kanan turun = rotasi
 * right-hand-rule positif sekitar X (sumbu kiri Y berputar naik ke
 * +Z); pitch positif = hidung turun = rotasi right-hand-rule positif
 * sekitar Y (sumbu depan X berputar turun ke -Z).
 *
 * Urutan komposisi rotasi WAJIB mengikuti konvensi Euler aerospace
 * standar (321: yaw - pitch - roll) yang sudah implisit dipakai oleh
 * formula roll=atan2(ay,az)/pitch=atan2(-ax,...) di atas: dari level,
 * PITCH diterapkan lebih dulu (sekitar sumbu Y level/yang belum
 * berputar, karena yaw=0 di sini), BARU ROLL diterapkan sekitar sumbu
 * X hasil-pitch (bukan sumbu X level) -- roll adalah rotasi paling
 * "dalam"/terakhir menuju body. Ini sebabnya roll bisa dihitung murni
 * dari ay,az saja (tidak tergantung ax) -- properti itu HANYA benar
 * untuk urutan pitch-dulu-baru-roll, bukan sebaliknya.
 *
 * Untuk membatalkan tilt (bawa vektor mag body-frame ke bidang datar),
 * body -> level pakai invers-nya: Ry(+pitch) * Rx(+roll) diterapkan ke
 * body-vec (roll dibatalkan dulu karena itu rotasi terluar/terakhir
 * saat membangun body, baru pitch):
 *
 *   Xh = bx*cos(pitch) + (by*sin(roll) + bz*cos(roll))*sin(pitch)
 *   Yh = by*cos(roll) - bz*sin(roll)
 *
 * Xh = komponen medan magnet ke arah depan-datar, Yh = ke arah
 * kiri-datar. Medan magnet horizontal selalu menunjuk ke utara
 * magnetik (bearing 0, searah jarum jam) -- proyeksikan ke arah
 * depan (bearing = heading) dan kiri (bearing = heading-90) pesawat:
 *
 *   Xh = H*cos(heading),  Yh = H*sin(heading)
 *   => heading = atan2(Yh, Xh)
 *
 * yang otomatis konsisten dengan konvensi atan2(east,north) dipakai
 * RTH_CalcDistanceBearing() (nav/rth.c) -- heading=0 saat menghadap
 * utara, naik searah jarum jam.
 *
 * Diverifikasi numerik terpisah dari derivasi simbolik di atas: 20000
 * sample acak (heading/roll/pitch acak, roll ±45°, pitch ±30°, medan
 * dengan komponen vertikal nonzero) dibangun lewat transform maju
 * body=Rx(-roll)*Ry(-pitch)*level lalu diumpankan balik ke rumus di
 * bawah -- error maksimum 0.000°. Urutan/rumus SEBELUM revisi ini
 * (Rx(-roll)*Ry(-pitch), bukan Ry(+pitch)*Rx(+roll)) mencapai error
 * hingga ~180° begitu roll DAN pitch nonzero bersamaan -- hanya
 * kebetulan benar untuk tilt satu sumbu (roll=0 atau pitch=0).
 */
float Compass_ComputeHeadingDeg(float mag_x, float mag_y, float mag_z,
                                float roll_deg, float pitch_deg)
{
    float roll_rad  = roll_deg  * DEG_TO_RAD_F;
    float pitch_rad = pitch_deg * DEG_TO_RAD_F;

    float cos_r = cosf(roll_rad);
    float sin_r = sinf(roll_rad);
    float cos_p = cosf(pitch_rad);
    float sin_p = sinf(pitch_rad);

    float xh = mag_x * cos_p + (mag_y * sin_r + mag_z * cos_r) * sin_p;
    float yh = mag_y * cos_r - mag_z * sin_r;

    float heading = atan2f(yh, xh) * RAD_TO_DEG_F;
    return wrap_deg_0_360(heading);
}

/* ------------------------------------------------------------------- */
/* Settings: grup "compass"                                             */
/* ------------------------------------------------------------------- */

/*
 * CATATAN PENTING (ditemukan saat menulis modul ini, di luar cakupan
 * task compass fusion): src/main.c saat ini TIDAK PERNAH memanggil
 * Settings_Init() atau Settings_FlashLoad() di mana pun -- ini bukan
 * regresi dari perubahan di file ini, seluruh subsistem Settings
 * (termasuk grup "mixer" yang menurut protocol.md Bagian 9 sudah
 * "final -- tunable") memang belum disambungkan ke boot sequence sama
 * sekali. Lihat juga kalibrasi-imu.md soal redesain persist-lewat-
 * Settings yang belum selesai.
 *
 * Registrasi di bawah tetap ditulis mengikuti kontrak settings.h apa
 * adanya (Compass_RegisterSettings() dipanggil dari main(), lihat
 * compass.h) supaya begitu Settings_Init()/Settings_FlashLoad()
 * disambungkan (task terpisah), grup "compass" langsung ikut aktif
 * tanpa perubahan lebih lanjut -- tapi sampai saat itu, field ini
 * TIDAK bisa diakses lewat CMD_SETTING_GET/SET dari web.
 *
 * SettingGetFn_t/SettingSetFn_t tidak punya parameter context, jadi
 * modul ini butuh pointer statis ke instance yang didaftarkan
 * (s_fc.compass_calib di main.c) -- caller WAJIB memberi pointer yang
 * hidup selama program berjalan (lihat komentar Compass_RegisterSettings()
 * di compass.h).
 */
static CompassCalib_t *s_settings_ctx = NULL;

static bool set_value_guarded(float *field, float value)
{
    if (s_settings_ctx == NULL || isnan(value) || isinf(value)) {
        return false;
    }
    *field = value;
    s_settings_ctx->valid = true;
    return true;
}

static SettingValue_t get_offset_x(void) { SettingValue_t v; v.number = s_settings_ctx ? (float)s_settings_ctx->offset_x : 0.0f; return v; }
static SettingValue_t get_offset_y(void) { SettingValue_t v; v.number = s_settings_ctx ? (float)s_settings_ctx->offset_y : 0.0f; return v; }
static SettingValue_t get_offset_z(void) { SettingValue_t v; v.number = s_settings_ctx ? (float)s_settings_ctx->offset_z : 0.0f; return v; }
static SettingValue_t get_scale_x(void)  { SettingValue_t v; v.number = s_settings_ctx ? s_settings_ctx->scale_x : 0.0f; return v; }
static SettingValue_t get_scale_y(void)  { SettingValue_t v; v.number = s_settings_ctx ? s_settings_ctx->scale_y : 0.0f; return v; }
static SettingValue_t get_scale_z(void)  { SettingValue_t v; v.number = s_settings_ctx ? s_settings_ctx->scale_z : 0.0f; return v; }

/* Offset disimpan int16 (raw LSB) -- schema wire NUMBER selalu f32,
 * jadi konversi bolak-balik lewat float di batas get/set, sama pola
 * dengan field NUMBER lain yang aslinya bukan float (lihat catatan
 * union SettingValue_t di settings.h). */
static bool set_offset_x(SettingValue_t v)
{
    if (s_settings_ctx == NULL || isnan(v.number) || isinf(v.number)) return false;
    s_settings_ctx->offset_x = (int16_t)v.number;
    s_settings_ctx->valid = true;
    return true;
}
static bool set_offset_y(SettingValue_t v)
{
    if (s_settings_ctx == NULL || isnan(v.number) || isinf(v.number)) return false;
    s_settings_ctx->offset_y = (int16_t)v.number;
    s_settings_ctx->valid = true;
    return true;
}
static bool set_offset_z(SettingValue_t v)
{
    if (s_settings_ctx == NULL || isnan(v.number) || isinf(v.number)) return false;
    s_settings_ctx->offset_z = (int16_t)v.number;
    s_settings_ctx->valid = true;
    return true;
}
static bool set_scale_x(SettingValue_t v) { return set_value_guarded(&s_settings_ctx->scale_x, v.number); }
static bool set_scale_y(SettingValue_t v) { return set_value_guarded(&s_settings_ctx->scale_y, v.number); }
static bool set_scale_z(SettingValue_t v) { return set_value_guarded(&s_settings_ctx->scale_z, v.number); }

static const SettingFieldDef_t kCompassFields[6] = {
    {
        .group = "compass", .key = "compass.offset_x", .label = "Offset X",
        .type = SETTING_TYPE_NUMBER, .has_min_max_step = true,
        .min = -4096.0f, .max = 4096.0f, .step = 1.0f,
        .unit = "lsb", .readonly_when_armed = false,
        .options = NULL, .option_count = 0,
        .get_fn = get_offset_x, .set_fn = set_offset_x,
    },
    {
        .group = "compass", .key = "compass.offset_y", .label = "Offset Y",
        .type = SETTING_TYPE_NUMBER, .has_min_max_step = true,
        .min = -4096.0f, .max = 4096.0f, .step = 1.0f,
        .unit = "lsb", .readonly_when_armed = false,
        .options = NULL, .option_count = 0,
        .get_fn = get_offset_y, .set_fn = set_offset_y,
    },
    {
        .group = "compass", .key = "compass.offset_z", .label = "Offset Z",
        .type = SETTING_TYPE_NUMBER, .has_min_max_step = true,
        .min = -4096.0f, .max = 4096.0f, .step = 1.0f,
        .unit = "lsb", .readonly_when_armed = false,
        .options = NULL, .option_count = 0,
        .get_fn = get_offset_z, .set_fn = set_offset_z,
    },
    {
        .group = "compass", .key = "compass.scale_x", .label = "Scale X",
        .type = SETTING_TYPE_NUMBER, .has_min_max_step = true,
        .min = 0.1f, .max = 3.0f, .step = 0.01f,
        .unit = NULL, .readonly_when_armed = false,
        .options = NULL, .option_count = 0,
        .get_fn = get_scale_x, .set_fn = set_scale_x,
    },
    {
        .group = "compass", .key = "compass.scale_y", .label = "Scale Y",
        .type = SETTING_TYPE_NUMBER, .has_min_max_step = true,
        .min = 0.1f, .max = 3.0f, .step = 0.01f,
        .unit = NULL, .readonly_when_armed = false,
        .options = NULL, .option_count = 0,
        .get_fn = get_scale_y, .set_fn = set_scale_y,
    },
    {
        .group = "compass", .key = "compass.scale_z", .label = "Scale Z",
        .type = SETTING_TYPE_NUMBER, .has_min_max_step = true,
        .min = 0.1f, .max = 3.0f, .step = 0.01f,
        .unit = NULL, .readonly_when_armed = false,
        .options = NULL, .option_count = 0,
        .get_fn = get_scale_z, .set_fn = set_scale_z,
    },
};

void Compass_RegisterSettings(CompassCalib_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    s_settings_ctx = ctx;
    for (size_t i = 0; i < (sizeof(kCompassFields) / sizeof(kCompassFields[0])); i++) {
        (void)Settings_RegisterField(&kCompassFields[i]);
    }
}
