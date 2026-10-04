#include "settings.h"
#include "command_handler.h"
#include "error.h"
#include <string.h>

/* ============================================================================
 * settings.c
 * Lihat settings.h untuk filosofi desain (registration-table dua level,
 * keterbatasan schema-tree, item terbuka backend penyimpanan permanen).
 *
 * PROPOSAL PAYLOAD -- BELUM DIKONFIRMASI TIM (sama sifatnya dengan
 * catatan di error.h utk CMD_ERROR): protocol.md Bagian 6 hanya
 * mendefinisikan bentuk byte RESPONS CMD_SETTING_SCHEMA_LIST, tidak
 * mendefinisikan payload REQUEST-nya (bagaimana web meminta halaman
 * tertentu), dan Bagian 5/7 sama sekali tidak memberi wire-format utk
 * payload CMD_SETTING_GET/CMD_SETTING_SET. Sebelum `core/protocol/registry.ts`
 * sisi web mengasumsikan bentuk di bawah, PERLU dikonfirmasi dulu --
 * kalau tim sepakat bentuk lain, HANYA fungsi-fungsi handle_* di file
 * ini yang perlu berubah, protocol.c/command_handler.c tidak perlu tahu.
 *
 *   CMD_SETTING_SCHEMA_LIST request : [page_index: u8]
 *     (halaman yang diminta; kirim 0 utk halaman pertama. Kalau
 *      payload_len==0, diperlakukan sama dengan page_index=0.)
 *   CMD_SETTING_SCHEMA_LIST response: sesuai protocol.md Bagian 6, tidak berubah
 *
 *   CMD_SETTING_GET request  : [key_len: u8][key: bytes]
 *   CMD_SETTING_GET response : [key_len: u8][key][type: u8][value...]
 *     value...: NUMBER=f32(4B), BOOL=u8(1B), ENUM/BITMASK=i32 LE(4B)
 *     (STRING tidak didukung, lihat settings.h)
 *
 *   CMD_SETTING_SET request  : [key_len: u8][key][type: u8][value...]
 *     (type disertakan supaya settings.c bisa memvalidasi payload_len
 *      cocok dengan tipe field TANPA harus lookup dulu, dan supaya
 *      mismatch tipe dari client bisa terdeteksi eksplisit)
 *   CMD_SETTING_SET response (sukses) : sama seperti CMD_SETTING_GET
 *     response, TAPI berisi nilai HASIL READBACK dari get_fn() setelah
 *     set_fn() dipanggil (bisa berbeda dari yang dikirim client kalau
 *     modul pemilik meng-clamp, mis. Mixer_SetAileronDifferential)
 *   CMD_SETTING_SET response (gagal): CMD_ERROR, lihat handle_setting_set()
 *
 *   CMD_SETTING_COMMIT request  : (kosong)
 *   CMD_SETTING_COMMIT response (sukses) : [status: u8] (selalu 0)
 *   CMD_SETTING_COMMIT response (gagal)  : CMD_ERROR ERROR_INTERNAL
 * ============================================================================ */

#define SETTINGS_NODE_BUF_SIZE (PROTOCOL_MAX_PAYLOAD + 1u)

static SettingFieldDef_t s_fields[SETTINGS_MAX_FIELDS];
static uint8_t           s_field_count;

/* Forward declarations -- didefinisikan di bawah, dipakai Settings_Init() */
static void handle_setting_schema_list(const protocol_frame_t *frame);
static void handle_setting_get(const protocol_frame_t *frame);
static void handle_setting_set(const protocol_frame_t *frame);
static void handle_setting_commit(const protocol_frame_t *frame);

/* ---------------------------------------------------------------------------
 * Backend penyimpanan permanen -- lihat catatan "ITEM TERBUKA" di
 * settings.h. Default weak SENGAJA gagal (return false), bukan
 * pura-pura sukses -- CMD_SETTING_COMMIT akan membalas CMD_ERROR
 * ERROR_INTERNAL sampai modul storage sungguhan meng-override kedua
 * fungsi ini dengan definisi kuat bernama sama.
 * ------------------------------------------------------------------------- */
__attribute__((weak)) bool Settings_FlashSave(void)
{
    return false;
}

__attribute__((weak)) bool Settings_FlashLoad(void)
{
    return false;
}

/* ---------------------------------------------------------------------------
 * Registry
 * ------------------------------------------------------------------------- */

static bool group_is_set(const char *group)
{
    return (group != 0) && (group[0] != '\0');
}

static const SettingFieldDef_t *find_field(const uint8_t *key, uint8_t key_len)
{
    for (uint8_t i = 0; i < s_field_count; i++) {
        size_t field_key_len = strlen(s_fields[i].key);
        if (field_key_len == (size_t)key_len &&
            memcmp(s_fields[i].key, key, key_len) == 0) {
            return &s_fields[i];
        }
    }
    return 0;
}

void Settings_Init(void)
{
    s_field_count = 0;

    CommandHandler_Register(CMD_SETTING_SCHEMA_LIST, handle_setting_schema_list);
    CommandHandler_Register(CMD_SETTING_GET, handle_setting_get);
    CommandHandler_Register(CMD_SETTING_SET, handle_setting_set);
    CommandHandler_Register(CMD_SETTING_COMMIT, handle_setting_commit);
}

bool Settings_RegisterField(const SettingFieldDef_t *def)
{
    if (def == 0 || def->key == 0 || def->get_fn == 0 || def->set_fn == 0) {
        return false;
    }
    if (strlen(def->key) > SETTINGS_MAX_KEY_LEN) {
        return false; /* tidak muat di field key_len u8 dgn batas SETTINGS_MAX_KEY_LEN */
    }
    {
        uint8_t klen = (uint8_t)strlen(def->key);
        if (find_field((const uint8_t *)def->key, klen) != 0) {
            return false; /* key sudah terdaftar -- cegah dua modul rebutan key sama */
        }
    }
    if (s_field_count >= SETTINGS_MAX_FIELDS) {
        return false; /* tabel penuh -- naikkan SETTINGS_MAX_FIELDS kalau perlu */
    }

    s_fields[s_field_count] = *def;
    s_field_count++;
    return true;
}

/* ---------------------------------------------------------------------------
 * Encoding schema (protocol.md Bagian 7) -- lihat keterbatasan dua-level
 * di settings.h.
 * ------------------------------------------------------------------------- */

static uint16_t encode_string(uint8_t *buf, const char *s, uint8_t max_len)
{
    uint8_t len = 0;
    if (s != 0) {
        size_t full_len = strlen(s);
        len = (full_len > (size_t)max_len) ? max_len : (uint8_t)full_len;
    }
    buf[0] = len;
    if (len > 0) {
        memcpy(&buf[1], s, len);
    }
    return (uint16_t)(1u + len);
}

/** @brief Encode satu field leaf (bukan grup) sesuai protocol.md Bagian 7.
 *         `buf` HARUS punya ruang minimal SETTINGS_NODE_BUF_SIZE --
 *         caller (build_top_nodes/handler) bertanggung jawab atas ini. */
static uint16_t encode_leaf(const SettingFieldDef_t *f, uint8_t *buf)
{
    uint16_t off = 0;

    off += encode_string(&buf[off], f->key, SETTINGS_MAX_KEY_LEN);
    off += encode_string(&buf[off], f->label, SETTINGS_MAX_LABEL_LEN);

    buf[off++] = (uint8_t)f->type;

    uint8_t flags = 0;
    if (f->has_min_max_step) {
        flags |= 0x01u;
    }
    if (f->readonly_when_armed) {
        flags |= 0x02u;
    }
    buf[off++] = flags;

    if (f->has_min_max_step) {
        memcpy(&buf[off], &f->min, sizeof(float));  off += (uint16_t)sizeof(float);
        memcpy(&buf[off], &f->max, sizeof(float));  off += (uint16_t)sizeof(float);
        memcpy(&buf[off], &f->step, sizeof(float)); off += (uint16_t)sizeof(float);
    }

    off += encode_string(&buf[off], f->unit, SETTINGS_MAX_UNIT_LEN);

    uint8_t opt_count = 0;
    if ((f->type == SETTING_TYPE_ENUM || f->type == SETTING_TYPE_BITMASK) && f->options != 0) {
        opt_count = f->option_count;
        if (opt_count > SETTINGS_MAX_OPTIONS) {
            opt_count = SETTINGS_MAX_OPTIONS; /* pagar keras terhadap def yang salah isi option_count */
        }
    }
    buf[off++] = opt_count;
    for (uint8_t i = 0; i < opt_count; i++) {
        int32_t v = f->options[i].value;
        memcpy(&buf[off], &v, sizeof(int32_t)); off += (uint16_t)sizeof(int32_t);
        off += encode_string(&buf[off], f->options[i].label, SETTINGS_MAX_LABEL_LEN);
    }

    buf[off++] = 0; /* child_count -- leaf tidak punya anak (lihat keterbatasan dua-level) */

    return off;
}

/** @brief Encode satu node grup (semua leaf ber-`group` sama, flat --
 *         lihat keterbatasan dua-level di settings.h). */
static uint16_t encode_group(const char *group_name, uint8_t *buf)
{
    uint16_t off = 0;

    off += encode_string(&buf[off], group_name, SETTINGS_MAX_KEY_LEN);
    off += encode_string(&buf[off], group_name, SETTINGS_MAX_LABEL_LEN); /* tidak ada label grup terpisah di v1 -- pakai nama key-nya sendiri */

    buf[off++] = (uint8_t)SETTING_TYPE_GROUP;
    buf[off++] = 0; /* flags: grup sendiri tidak readonly/has_min_max_step, berlaku per-child */
    off += encode_string(&buf[off], 0, 0); /* unit kosong utk grup */
    buf[off++] = 0; /* option_count grup selalu 0 */

    uint8_t child_count = 0;
    for (uint8_t i = 0; i < s_field_count; i++) {
        if (group_is_set(s_fields[i].group) && strcmp(s_fields[i].group, group_name) == 0) {
            child_count++;
        }
    }
    buf[off++] = child_count;

    for (uint8_t i = 0; i < s_field_count; i++) {
        if (group_is_set(s_fields[i].group) && strcmp(s_fields[i].group, group_name) == 0) {
            /* Catatan batas ukuran: kalau satu grup punya anak sangat
             * banyak sampai off melebihi SETTINGS_NODE_BUF_SIZE, ini
             * buffer overflow. Tidak ada guard runtime di sini (buffer
             * caller sudah PROTOCOL_MAX_PAYLOAD+1 byte, dan v1 hanya
             * py 2 grup dgn <=10 child masing-masing) -- TODO kalau
             * jumlah field per grup diperkirakan bisa jauh lebih besar
             * ke depannya, tambahkan bound check eksplisit di sini.
             */
            off += encode_leaf(&s_fields[i], &buf[off]);
        }
    }

    return off;
}

/* ---------------------------------------------------------------------------
 * Top-level node list: grup (satu entri per nama grup unik, urutan
 * kemunculan pertama) + field ungrouped (langsung sbg leaf top-level).
 * Dibangun ulang tiap request (lihat catatan di settings.h kenapa ini
 * tidak di-cache).
 * ------------------------------------------------------------------------- */

typedef struct {
    bool        is_group;
    uint8_t     field_index; /* valid kalau !is_group */
    const char *group_name;  /* valid kalau is_group */
} TopNode_t;

static uint8_t build_top_nodes(TopNode_t *out, uint8_t max_out)
{
    uint8_t n = 0;

    for (uint8_t i = 0; i < s_field_count && n < max_out; i++) {
        const SettingFieldDef_t *f = &s_fields[i];

        if (!group_is_set(f->group)) {
            out[n].is_group = false;
            out[n].field_index = i;
            n++;
            continue;
        }

        bool seen = false;
        for (uint8_t j = 0; j < n; j++) {
            if (out[j].is_group && strcmp(out[j].group_name, f->group) == 0) {
                seen = true;
                break;
            }
        }
        if (!seen) {
            out[n].is_group = true;
            out[n].group_name = f->group;
            n++;
        }
    }

    return n;
}

/* ---------------------------------------------------------------------------
 * Handler: CMD_SETTING_SCHEMA_LIST
 * ------------------------------------------------------------------------- */

static void handle_setting_schema_list(const protocol_frame_t *frame)
{
    uint8_t requested_page = (frame->payload_len >= 1u) ? frame->payload[0] : 0u;

    TopNode_t nodes[SETTINGS_MAX_FIELDS];
    uint8_t node_count = build_top_nodes(nodes, (uint8_t)(sizeof(nodes) / sizeof(nodes[0])));

    uint8_t node_i = 0;
    uint8_t current_page = 0;

    while (1) {
        uint8_t page_payload[PROTOCOL_MAX_PAYLOAD];
        uint8_t node_buf[SETTINGS_NODE_BUF_SIZE];
        uint16_t page_off = 3; /* header: page_index, has_more, field_count */
        uint8_t field_count_this_page = 0;

        while (node_i < node_count) {
            uint16_t node_len;

            if (nodes[node_i].is_group) {
                node_len = encode_group(nodes[node_i].group_name, node_buf);
            } else {
                node_len = encode_leaf(&s_fields[nodes[node_i].field_index], node_buf);
            }

            if (node_len > (uint16_t)PROTOCOL_MAX_PAYLOAD) {
                /* satu node sendirian sudah melebihi kapasitas satu
                 * frame -- tidak representable di protokol ini
                 * (lihat catatan keterbatasan settings.h). Lewati node
                 * ini daripada mengirim frame yang korup. */
                node_i++;
                continue;
            }
            if (field_count_this_page > 0 && (page_off + node_len) > (uint16_t)PROTOCOL_MAX_PAYLOAD) {
                break; /* halaman ini penuh, node_i TIDAK dimajukan -- lanjut di halaman berikutnya */
            }

            memcpy(&page_payload[page_off], node_buf, node_len);
            page_off += node_len;
            field_count_this_page++;
            node_i++;
        }

        bool has_more = (node_i < node_count);

        if (current_page == requested_page) {
            page_payload[0] = current_page;
            page_payload[1] = has_more ? 1u : 0u;
            page_payload[2] = field_count_this_page;
            Protocol_SendFrame(CMD_SETTING_SCHEMA_LIST, frame->request_id, page_payload, (uint8_t)page_off);
            return;
        }

        if (!has_more) {
            /* requested_page diminta melebihi jumlah halaman yang ada
             * (termasuk kasus node_count==0 dgn requested_page>0) */
            Error_Send(frame->request_id, ERROR_INVALID_PAYLOAD_LENGTH, CMD_SETTING_SCHEMA_LIST);
            return;
        }

        current_page++;
    }
}

/* ---------------------------------------------------------------------------
 * Decode/encode value sesuai type -- dipakai handle_setting_get/set.
 * ------------------------------------------------------------------------- */

static uint8_t value_wire_size(SettingType_t type)
{
    switch (type) {
        case SETTING_TYPE_NUMBER:  return 4u;
        case SETTING_TYPE_BOOL:    return 1u;
        case SETTING_TYPE_ENUM:    return 4u;
        case SETTING_TYPE_BITMASK: return 4u;
        default:                   return 0u; /* STRING/GROUP tidak didukung di jalur GET/SET generik ini */
    }
}

static void encode_value(SettingType_t type, SettingValue_t value, uint8_t *buf)
{
    switch (type) {
        case SETTING_TYPE_NUMBER:
            memcpy(buf, &value.number, sizeof(float));
            break;
        case SETTING_TYPE_BOOL:
            buf[0] = value.boolean ? 1u : 0u;
            break;
        case SETTING_TYPE_ENUM:
        case SETTING_TYPE_BITMASK:
            memcpy(buf, &value.enum_val, sizeof(int32_t));
            break;
        default:
            break;
    }
}

static bool decode_value(SettingType_t type, const uint8_t *buf, SettingValue_t *out)
{
    switch (type) {
        case SETTING_TYPE_NUMBER:
            memcpy(&out->number, buf, sizeof(float));
            return true;
        case SETTING_TYPE_BOOL:
            out->boolean = (buf[0] != 0u);
            return true;
        case SETTING_TYPE_ENUM:
        case SETTING_TYPE_BITMASK:
            memcpy(&out->enum_val, buf, sizeof(int32_t));
            return true;
        default:
            return false;
    }
}

/** @brief Bangun payload response GET/SET sukses: [key_len][key][type][value...]. */
static uint8_t build_get_response(const SettingFieldDef_t *f, uint8_t *out)
{
    uint16_t off = 0;
    off += encode_string(&out[off], f->key, SETTINGS_MAX_KEY_LEN);
    out[off++] = (uint8_t)f->type;
    encode_value(f->type, f->get_fn(), &out[off]);
    off += value_wire_size(f->type);
    return (uint8_t)off;
}

/* ---------------------------------------------------------------------------
 * Handler: CMD_SETTING_GET
 * ------------------------------------------------------------------------- */

static void handle_setting_get(const protocol_frame_t *frame)
{
    if (frame->payload_len < 1u) {
        Error_Send(frame->request_id, ERROR_INVALID_PAYLOAD_LENGTH, CMD_SETTING_GET);
        return;
    }
    uint8_t key_len = frame->payload[0];
    if ((uint16_t)(1u + key_len) > frame->payload_len) {
        Error_Send(frame->request_id, ERROR_INVALID_PAYLOAD_LENGTH, CMD_SETTING_GET);
        return;
    }

    const SettingFieldDef_t *f = find_field(&frame->payload[1], key_len);
    if (f == 0) {
        Error_Send(frame->request_id, ERROR_SETTING_KEY_NOT_FOUND, CMD_SETTING_GET);
        return;
    }

    uint8_t response[PROTOCOL_MAX_PAYLOAD];
    uint8_t response_len = build_get_response(f, response);
    Protocol_SendFrame(CMD_SETTING_GET, frame->request_id, response, response_len);
}

/* ---------------------------------------------------------------------------
 * Handler: CMD_SETTING_SET
 * ------------------------------------------------------------------------- */

static void handle_setting_set(const protocol_frame_t *frame)
{
    if (frame->payload_len < 2u) {
        Error_Send(frame->request_id, ERROR_INVALID_PAYLOAD_LENGTH, CMD_SETTING_SET);
        return;
    }
    uint8_t key_len = frame->payload[0];
    uint16_t off = (uint16_t)(1u + key_len);
    if (off >= frame->payload_len) {
        Error_Send(frame->request_id, ERROR_INVALID_PAYLOAD_LENGTH, CMD_SETTING_SET);
        return;
    }

    const SettingFieldDef_t *f = find_field(&frame->payload[1], key_len);
    if (f == 0) {
        Error_Send(frame->request_id, ERROR_SETTING_KEY_NOT_FOUND, CMD_SETTING_SET);
        return;
    }

    if (f->readonly_when_armed && Armed_IsArmed()) {
        Error_Send(frame->request_id, ERROR_ARMED_REJECTED, CMD_SETTING_SET);
        return;
    }

    SettingType_t wire_type = (SettingType_t)frame->payload[off];
    off += 1u;
    if (wire_type != f->type) {
        Error_Send(frame->request_id, ERROR_INVALID_PAYLOAD_LENGTH, CMD_SETTING_SET);
        return;
    }

    uint8_t value_size = value_wire_size(f->type);
    if (value_size == 0u || (uint16_t)(off + value_size) > frame->payload_len) {
        Error_Send(frame->request_id, ERROR_INVALID_PAYLOAD_LENGTH, CMD_SETTING_SET);
        return;
    }

    SettingValue_t value;
    if (!decode_value(f->type, &frame->payload[off], &value)) {
        Error_Send(frame->request_id, ERROR_INVALID_PAYLOAD_LENGTH, CMD_SETTING_SET);
        return;
    }

    if (f->type == SETTING_TYPE_NUMBER && f->has_min_max_step) {
        if (value.number < f->min || value.number > f->max) {
            Error_Send(frame->request_id, ERROR_SETTING_VALUE_OUT_OF_RANGE, CMD_SETTING_SET);
            return;
        }
    }

    if (!f->set_fn(value)) {
        /* modul pemilik menolak (mis. NaN/Inf) -- reuse kode error yang
         * ada, lihat catatan di settings.h soal SettingSetFn_t */
        Error_Send(frame->request_id, ERROR_SETTING_VALUE_OUT_OF_RANGE, CMD_SETTING_SET);
        return;
    }

    uint8_t response[PROTOCOL_MAX_PAYLOAD];
    uint8_t response_len = build_get_response(f, response); /* readback -- lihat catatan di settings.h */
    Protocol_SendFrame(CMD_SETTING_SET, frame->request_id, response, response_len);
}

/* ---------------------------------------------------------------------------
 * Handler: CMD_SETTING_COMMIT
 * ------------------------------------------------------------------------- */

static void handle_setting_commit(const protocol_frame_t *frame)
{
    if (Settings_FlashSave()) {
        uint8_t status = 0;
        Protocol_SendFrame(CMD_SETTING_COMMIT, frame->request_id, &status, 1u);
    } else {
        /* Default weak Settings_FlashSave() SELALU return false sampai
         * backend penyimpanan permanen diputuskan & di-override --
         * lihat catatan "ITEM TERBUKA" di settings.h. */
        Error_Send(frame->request_id, ERROR_INTERNAL, CMD_SETTING_COMMIT);
    }
}
