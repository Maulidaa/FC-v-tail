#include "usb_cdc_if.h"
#include "bsp_pinmap.h"
#include <string.h>

/* ============================================================================
 * usb_cdc_if.c
 * Implementasi bare-metal USB OTG FS device mode, kelas CDC-ACM tunggal
 * (non-composite -- bDeviceClass=0x02 di device descriptor, TIDAK pakai
 * Interface Association Descriptor karena tidak perlu untuk single CDC
 * function). Pola register mengikuti urutan baku RM0383 Bab 22 (OTG_FS)
 * yang juga dipakai ST USB Device Library -- ditulis ulang dari nol di
 * sini tanpa dependency ke library tsb, sesuai gaya bare-metal modul lain.
 * ============================================================================ */

#define USBx   USB_OTG_FS

/* Macro turunan berikut biasanya datang dari driver USB Device Library ST
 * (stm32f4xx_ll_usb.h), tapi modul ini sengaja ditulis bare-metal tanpa
 * dependency ke situ. Base offset-nya sendiri (USB_OTG_DEVICE_BASE dkk)
 * sudah ada di CMSIS device header, jadi tinggal disusun ulang di sini. */
#define USBx_BASE       USB_OTG_FS_PERIPH_BASE
#define USBx_DEVICE     ((USB_OTG_DeviceTypeDef *)(USBx_BASE + USB_OTG_DEVICE_BASE))
#define USBx_INEP(i)    ((USB_OTG_INEndpointTypeDef *)(USBx_BASE + USB_OTG_IN_ENDPOINT_BASE + (i) * USB_OTG_EP_REG_SIZE))
#define USBx_OUTEP(i)   ((USB_OTG_OUTEndpointTypeDef *)(USBx_BASE + USB_OTG_OUT_ENDPOINT_BASE + (i) * USB_OTG_EP_REG_SIZE))

/* Endpoint 0 (control) selalu ada. CDC butuh 2 endpoint tambahan:
 * EP1 (bulk data IN/OUT, dua arah = dua hardware endpoint object beda)
 * EP2 IN (notification interrupt, dideklarasikan di descriptor tapi tidak
 * pernah benar2 dipakai kirim data -- cukup umum untuk CDC-ACM minimal,
 * host tidak mewajibkan traffic aktif di endpoint ini). */
#define EP0_MPS   64u
#define EP1_MPS   USB_CDC_BULK_MPS
#define EP2_MPS   8u

/* Alokasi FIFO (dalam word 32-bit) -- total FS OTG core STM32F411 = 320
 * word (1.25KB). RX shared utk semua OUT ep + SETUP, TX terpisah per IN ep. */
#define RXFIFO_SIZE_WORDS    128u
#define TX0FIFO_SIZE_WORDS   32u
#define TX1FIFO_SIZE_WORDS   64u
#define TX2FIFO_SIZE_WORDS   32u

/* ---------------------------------------------------------------------------
 * Deskriptor USB (statis, disusun manual sesuai USB CDC-ACM spec 1.10)
 * ------------------------------------------------------------------------- */

static const uint8_t s_device_descriptor[18] = {
    18, 0x01,             /* bLength, bDescriptorType=DEVICE */
    0x00, 0x02,            /* bcdUSB = 2.00 */
    0x02, 0x00, 0x00,      /* bDeviceClass=CDC, SubClass=0, Protocol=0 */
    EP0_MPS,               /* bMaxPacketSize0 */
    (uint8_t)(USB_VID & 0xFFu), (uint8_t)(USB_VID >> 8),
    (uint8_t)(USB_PID & 0xFFu), (uint8_t)(USB_PID >> 8),
    0x00, 0x01,            /* bcdDevice = 1.00 */
    0x01,                  /* iManufacturer */
    0x02,                  /* iProduct */
    0x00,                  /* iSerialNumber (tidak dipakai) */
    0x01                   /* bNumConfigurations */
};

/* Total 67 byte: config(9) + IF0(9) + CDC-header(5) + call-mgmt(5) +
 * ACM(4) + union(5) + EP-notify(7) + IF1(9) + EP-out(7) + EP-in(7) */
static const uint8_t s_config_descriptor[67] = {
    /* Configuration descriptor */
    9, 0x02, 67, 0x00, 0x02, 0x01, 0x00, 0xC0, 0x32,
    /* Interface 0: Communication Class */
    9, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,
    /* CDC Header Functional Descriptor */
    5, 0x24, 0x00, 0x10, 0x01,
    /* CDC Call Management Functional Descriptor */
    5, 0x24, 0x01, 0x00, 0x01,
    /* CDC ACM Functional Descriptor */
    4, 0x24, 0x02, 0x02,
    /* CDC Union Functional Descriptor */
    5, 0x24, 0x06, 0x00, 0x01,
    /* Endpoint: notification IN (EP2 IN, interrupt, tidak pernah dipakai aktif) */
    7, 0x05, 0x82, 0x03, (uint8_t)(EP2_MPS & 0xFFu), 0x00, 0x10,
    /* Interface 1: Data Class */
    9, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,
    /* Endpoint: bulk OUT (EP1 OUT, host->device) */
    7, 0x05, 0x01, 0x02, (uint8_t)(EP1_MPS & 0xFFu), 0x00, 0x00,
    /* Endpoint: bulk IN (EP1 IN, device->host) */
    7, 0x05, 0x81, 0x02, (uint8_t)(EP1_MPS & 0xFFu), 0x00, 0x00,
};

static const uint8_t s_string_lang[4] = { 4, 0x03, 0x09, 0x04 }; /* 0x0409 = English (US) */

/* "TeamFC" dalam UTF-16LE, dibungkus header string descriptor */
static const uint8_t s_string_mfg[14] = {
    14, 0x03,
    'T', 0, 'e', 0, 'a', 0, 'm', 0, 'F', 0, 'C', 0
};

/* "FC-CDC" dalam UTF-16LE */
static const uint8_t s_string_prod[14] = {
    14, 0x03,
    'F', 0, 'C', 0, '-', 0, 'C', 0, 'D', 0, 'C', 0
};

/* ---------------------------------------------------------------------------
 * State device (setup transfer, konfigurasi, line coding CDC)
 * ------------------------------------------------------------------------- */

typedef enum {
    CTRL_STAGE_IDLE = 0,
    CTRL_STAGE_DATA_IN,
    CTRL_STAGE_STATUS_IN,
    CTRL_STAGE_STATUS_OUT,
} ctrl_stage_t;

static volatile ctrl_stage_t s_ctrl_stage = CTRL_STAGE_IDLE;
static volatile int s_device_configured = 0;
/* DTR dari host (SET_CONTROL_LINE_STATE bit0) -- dipakai untuk tahu apakah
 * ADA program host yang membuka port. Bukan untuk kontrol elektris. Lihat
 * BSP_USB_CDC_IsHostOpen() di usb_cdc_if.h. */
static volatile int s_host_dtr = 0;
static volatile uint8_t s_pending_address = 0; /* SET_ADDRESS baru efektif setelah status stage (lihat USB spec 9.4.6) */

/* Buffer data IN control transfer (deskriptor dikirim dari sini, dipecah
 * per paket EP0_MPS oleh state machine kalau lebih panjang dari itu). */
static const uint8_t *s_ctrl_tx_ptr;
static uint16_t s_ctrl_tx_remaining;

/* CDC line coding (SET/GET_LINE_CODING) -- disimpan supaya host driver
 * senang (ada yang query balik nilai yang baru di-set), tapi TIDAK
 * mengubah apa pun secara elektris (virtual COM port murni). */
static uint8_t s_line_coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08 }; /* default 115200 8N1 */

/* ---------------------------------------------------------------------------
 * Ring buffer RX (pola sama seperti bsp_uart.c)
 * ------------------------------------------------------------------------- */

typedef struct {
    volatile uint8_t buf[USB_CDC_RX_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} usb_ring_t;

static usb_ring_t s_rx_ring;

static void ring_push(usb_ring_t *r, uint8_t byte)
{
    uint16_t next = (uint16_t)((r->head + 1u) % USB_CDC_RX_BUF_SIZE);
    if (next == r->tail) {
        return; /* penuh -- byte terbaru di-drop, sama kebijakan seperti bsp_uart.c */
    }
    r->buf[r->head] = byte;
    r->head = next;
}

static uint16_t ring_available(const usb_ring_t *r)
{
    return (uint16_t)((r->head - r->tail) % USB_CDC_RX_BUF_SIZE);
}

static int ring_pop(usb_ring_t *r, uint8_t *out)
{
    if (r->head == r->tail) {
        return 0;
    }
    *out = r->buf[r->tail];
    r->tail = (uint16_t)((r->tail + 1u) % USB_CDC_RX_BUF_SIZE);
    return 1;
}

/* ---------------------------------------------------------------------------
 * Helper akses FIFO & endpoint
 * ------------------------------------------------------------------------- */

static volatile uint32_t *fifo_ptr(uint8_t ep_num)
{
    return (volatile uint32_t *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE
                                  + (uint32_t)ep_num * USB_OTG_FIFO_SIZE);
}

static void write_packet_to_fifo(uint8_t ep_num, const uint8_t *data, uint16_t len)
{
    volatile uint32_t *fifo = fifo_ptr(ep_num);
    uint16_t words = (uint16_t)((len + 3u) / 4u);
    for (uint16_t i = 0; i < words; i++) {
        uint32_t w;
        memcpy(&w, &data[i * 4u], (len - i * 4u) >= 4u ? 4u : (len - i * 4u));
        *fifo = w;
    }
}

/* Mulai transfer 1 paket (<=MPS) di endpoint IN tertentu. */
static void ep_in_transfer(uint8_t ep_num, const uint8_t *data, uint16_t len)
{
    USB_OTG_INEndpointTypeDef *in_ep = USBx_INEP(ep_num);

    in_ep->DIEPTSIZ = (1u << USB_OTG_DIEPTSIZ_PKTCNT_Pos) | len;
    in_ep->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;

    if (len > 0u) {
        write_packet_to_fifo(ep_num, data, len);
    }
}

/* Re-arm endpoint OUT untuk menerima 1 paket lagi (dipanggil ulang tiap
 * kali selesai proses 1 paket, supaya endpoint selalu siap terima). */
static void ep_out_prepare(uint8_t ep_num, uint16_t mps)
{
    USB_OTG_OUTEndpointTypeDef *out_ep = USBx_OUTEP(ep_num);
    out_ep->DOEPTSIZ = (1u << USB_OTG_DOEPTSIZ_PKTCNT_Pos) | mps;
    out_ep->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
}

/* EP0 OUT butuh arming khusus (STUPCNT utk buffer setup packet). */
static void ep0_out_prepare(void)
{
    USBx_OUTEP(0)->DOEPTSIZ = (3u << USB_OTG_DOEPTSIZ_STUPCNT_Pos)
                            | (1u << USB_OTG_DOEPTSIZ_PKTCNT_Pos)
                            | EP0_MPS;
    USBx_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
}

/* Kirim data control-transfer (dipecah otomatis per EP0_MPS). */
static void ctrl_send(const uint8_t *data, uint16_t len, uint16_t requested_len)
{
    uint16_t send_len = (len < requested_len) ? len : requested_len;
    s_ctrl_tx_ptr = data;
    s_ctrl_tx_remaining = send_len;

    uint16_t chunk = (send_len > EP0_MPS) ? EP0_MPS : send_len;
    s_ctrl_stage = CTRL_STAGE_DATA_IN;
    ep_in_transfer(0, s_ctrl_tx_ptr, chunk);
    s_ctrl_tx_ptr += chunk;
    s_ctrl_tx_remaining = (uint16_t)(send_len - chunk);
}

static void ctrl_send_status(void)
{
    s_ctrl_stage = CTRL_STAGE_STATUS_IN;
    ep_in_transfer(0, NULL, 0); /* ZLP status stage */
}

/* ---------------------------------------------------------------------------
 * Handling SETUP packet (control transfer standar + class-specific CDC)
 * ------------------------------------------------------------------------- */

#define REQ_GET_STATUS         0x00u
#define REQ_SET_ADDRESS        0x05u
#define REQ_GET_DESCRIPTOR     0x06u
#define REQ_SET_CONFIGURATION  0x09u
#define REQ_SET_LINE_CODING    0x20u /* CDC class-specific */
#define REQ_GET_LINE_CODING    0x21u
#define REQ_SET_CONTROL_LINE_STATE 0x22u

static void handle_setup(const uint8_t *setup)
{
    uint8_t  bmRequestType = setup[0];
    uint8_t  bRequest      = setup[1];
    uint16_t wValue        = (uint16_t)(setup[2] | (setup[3] << 8));
    uint16_t wLength       = (uint16_t)(setup[6] | (setup[7] << 8));

    uint8_t req_type_class = (uint8_t)((bmRequestType >> 5) & 0x3u); /* 0=standard,1=class,2=vendor */

    if (req_type_class == 0u) {
        /* --- Standard device/interface/endpoint request --- */
        switch (bRequest) {
            case REQ_GET_DESCRIPTOR: {
                uint8_t desc_type = (uint8_t)(wValue >> 8);
                uint8_t desc_idx  = (uint8_t)(wValue & 0xFFu);
                if (desc_type == 0x01u) { /* DEVICE */
                    ctrl_send(s_device_descriptor, sizeof(s_device_descriptor), wLength);
                } else if (desc_type == 0x02u) { /* CONFIGURATION */
                    ctrl_send(s_config_descriptor, sizeof(s_config_descriptor), wLength);
                } else if (desc_type == 0x03u) { /* STRING */
                    if (desc_idx == 0u) {
                        ctrl_send(s_string_lang, sizeof(s_string_lang), wLength);
                    } else if (desc_idx == 1u) {
                        ctrl_send(s_string_mfg, sizeof(s_string_mfg), wLength);
                    } else if (desc_idx == 2u) {
                        ctrl_send(s_string_prod, sizeof(s_string_prod), wLength);
                    } else {
                        USBx_INEP(0)->DIEPCTL |= USB_OTG_DIEPCTL_STALL; /* index tidak dikenal */
                    }
                } else {
                    USBx_INEP(0)->DIEPCTL |= USB_OTG_DIEPCTL_STALL; /* tipe deskriptor tidak didukung */
                }
                break;
            }
            case REQ_SET_ADDRESS:
                /* Alamat baru baru efektif SETELAH status stage (spec 9.4.6) --
                 * disimpan dulu, diterapkan ke DCFG di ISR IN complete utk EP0. */
                s_pending_address = (uint8_t)(wValue & 0x7Fu);
                ctrl_send_status();
                break;
            case REQ_SET_CONFIGURATION:
                s_device_configured = 1;
                /* Aktifkan endpoint CDC (EP1 IN/OUT bulk, EP2 IN interrupt) */
                USBx_INEP(1)->DIEPCTL = (EP1_MPS) | (2u << USB_OTG_DIEPCTL_EPTYP_Pos)
                                      | USB_OTG_DIEPCTL_SD0PID_SEVNFRM | USB_OTG_DIEPCTL_USBAEP
                                      | (1u << USB_OTG_DIEPCTL_TXFNUM_Pos);
                USBx_OUTEP(1)->DOEPCTL = (EP1_MPS) | (2u << USB_OTG_DOEPCTL_EPTYP_Pos)
                                       | USB_OTG_DOEPCTL_SD0PID_SEVNFRM | USB_OTG_DOEPCTL_USBAEP;
                USBx_INEP(2)->DIEPCTL = (EP2_MPS) | (3u << USB_OTG_DIEPCTL_EPTYP_Pos)
                                      | USB_OTG_DIEPCTL_SD0PID_SEVNFRM | USB_OTG_DIEPCTL_USBAEP
                                      | (2u << USB_OTG_DIEPCTL_TXFNUM_Pos);
                USBx_DEVICE->DAINTMSK |= (1u << 1) | (1u << (16 + 1)); /* EP1 IN & OUT */
                ep_out_prepare(1, EP1_MPS); /* mulai siap terima data bulk dari host */
                ctrl_send_status();
                break;
            case REQ_GET_STATUS: {
                static const uint8_t status_reply[2] = { 0x00, 0x00 };
                ctrl_send(status_reply, sizeof(status_reply), wLength);
                break;
            }
            default:
                USBx_INEP(0)->DIEPCTL |= USB_OTG_DIEPCTL_STALL; /* request standar tak dikenal/tak didukung */
                break;
        }
    } else if (req_type_class == 1u) {
        /* --- CDC class-specific request --- */
        switch (bRequest) {
            case REQ_SET_LINE_CODING:
                /* Data stage OUT 7 byte akan datang lewat RXFLVL/EP0 OUT --
                 * disederhanakan: langsung ACK status, isi s_line_coding
                 * diabaikan (virtual COM, tidak ada UART fisik yang perlu
                 * di-reconfigure). Kalau host benar2 mengirim data stage,
                 * itu akan diterima sebagai OUT packet biasa di ISR RXFLVL
                 * dan cukup di-drain tanpa efek samping. */
                ctrl_send_status();
                break;
            case REQ_GET_LINE_CODING:
                ctrl_send(s_line_coding, sizeof(s_line_coding), wLength);
                break;
            case REQ_SET_CONTROL_LINE_STATE:
                /* DTR/RTS dari host (bit0/bit1 di wValue) -- tidak ada efek
                 * elektris (virtual COM), cukup ACK supaya driver host puas.
                 * DTR dicatat (bit0) sebagai penanda "ada host yang membuka
                 * port" untuk push telemetri, lihat BSP_USB_CDC_IsHostOpen(). */
                s_host_dtr = (int)(wValue & 0x1u);
                ctrl_send_status();
                break;
            default:
                USBx_INEP(0)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
                break;
        }
    } else {
        USBx_INEP(0)->DIEPCTL |= USB_OTG_DIEPCTL_STALL; /* vendor request tidak didukung */
    }
}

/* ---------------------------------------------------------------------------
 * Init inti OTG FS core (device mode, FS PHY internal, FIFO sizing)
 * ------------------------------------------------------------------------- */

static void otg_core_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
    __asm volatile ("nop");
    __asm volatile ("nop");

    /* Core soft reset */
    while (!(USBx->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL)) { }
    USBx->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
    while (USBx->GRSTCTL & USB_OTG_GRSTCTL_CSRST) { }

    /* PHY FS internal, power up transceiver, VBUS sensing DIMATIKAN
     * (lihat catatan PA9 di header) */
    USBx->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;
    USBx->GCCFG = USB_OTG_GCCFG_NOVBUSSENS | USB_OTG_GCCFG_PWRDWN;
    for (volatile int d = 0; d < 200000; d++) { } /* stabilisasi transceiver */

    /* Paksa device mode (bukan OTG dual-role) */
    USBx->GUSBCFG &= ~USB_OTG_GUSBCFG_FHMOD;
    USBx->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
    for (volatile int d = 0; d < 200000; d++) { } /* wajib delay setelah force mode, RM0383 */

    /* Turnaround time -- AHB clock 96MHz -> TRDT=0x6 (tabel RM0383) */
    USBx->GUSBCFG = (USBx->GUSBCFG & ~USB_OTG_GUSBCFG_TRDT) | (0x6u << USB_OTG_GUSBCFG_TRDT_Pos);

    /* Device speed = full-speed (PHY internal) */
    USBx_DEVICE->DCFG = (USBx_DEVICE->DCFG & ~USB_OTG_DCFG_DSPD) | (0x3u << USB_OTG_DCFG_DSPD_Pos);

    /* Flush FIFO (semua TX + RX) sebelum sizing ulang */
    USBx->GRSTCTL = (USBx->GRSTCTL & ~USB_OTG_GRSTCTL_TXFNUM) | (0x10u << USB_OTG_GRSTCTL_TXFNUM_Pos) | USB_OTG_GRSTCTL_TXFFLSH;
    while (USBx->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) { }
    USBx->GRSTCTL |= USB_OTG_GRSTCTL_RXFFLSH;
    while (USBx->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) { }

    /* FIFO sizing -- lihat komentar alokasi word di atas file */
    USBx->GRXFSIZ = RXFIFO_SIZE_WORDS;
    USBx->DIEPTXF0_HNPTXFSIZ = (TX0FIFO_SIZE_WORDS << 16) | RXFIFO_SIZE_WORDS;
    USBx->DIEPTXF[0] = (TX1FIFO_SIZE_WORDS << 16) | (RXFIFO_SIZE_WORDS + TX0FIFO_SIZE_WORDS);
    USBx->DIEPTXF[1] = (TX2FIFO_SIZE_WORDS << 16) | (RXFIFO_SIZE_WORDS + TX0FIFO_SIZE_WORDS + TX1FIFO_SIZE_WORDS);

    /* Endpoint 0 IN/OUT: control, MPS 64 */
    USBx_INEP(0)->DIEPCTL = (0x0u << USB_OTG_DIEPCTL_MPSIZ_Pos); /* 00 = 64 byte, EPTYP control default 0 */
    USBx_OUTEP(0)->DOEPCTL = 0;

    /* Unmask interrupt endpoint (EP0 IN/OUT selalu aktif) */
    USBx_DEVICE->DAINTMSK = (1u << 0) | (1u << (16 + 0));
    USBx_DEVICE->DIEPMSK  = USB_OTG_DIEPMSK_XFRCM;
    USBx_DEVICE->DOEPMSK  = USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_STUPM;

    /* Interrupt global core */
    USBx->GINTSTS = 0xFFFFFFFFu; /* clear semua flag pending (W1C) */
    USBx->GINTMSK = USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM
                  | USB_OTG_GINTMSK_RXFLVLM | USB_OTG_GINTMSK_IEPINT
                  | USB_OTG_GINTMSK_OEPINT;

    USBx->GAHBCFG |= USB_OTG_GAHBCFG_GINT;

    NVIC_SetPriority(OTG_FS_IRQn, 4);
    NVIC_EnableIRQ(OTG_FS_IRQn);

    /* Soft-connect: lepas pull-up disable, host mulai deteksi & enumerasi */
    USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS;
}

/* ---------------------------------------------------------------------------
 * API publik
 * ------------------------------------------------------------------------- */

void BSP_USB_CDC_Init(void)
{
    memset((void *)&s_rx_ring, 0, sizeof(s_rx_ring));
    s_device_configured = 0;
    s_host_dtr = 0;
    s_ctrl_stage = CTRL_STAGE_IDLE;

    otg_core_init();
}

int BSP_USB_CDC_IsConfigured(void)
{
    return s_device_configured;
}

int BSP_USB_CDC_IsHostOpen(void)
{
    return s_device_configured && s_host_dtr;
}

uint16_t BSP_USB_CDC_Available(void)
{
    return ring_available(&s_rx_ring);
}

int BSP_USB_CDC_ReadByte(uint8_t *out)
{
    return ring_pop(&s_rx_ring, out);
}

int BSP_USB_CDC_WriteBuf(const uint8_t *data, uint16_t len)
{
    if (!s_device_configured) {
        return 0;
    }

    uint16_t sent = 0;
    while (sent < len) {
        uint16_t chunk = (uint16_t)(len - sent);
        if (chunk > EP1_MPS) {
            chunk = EP1_MPS;
        }

        /* Tunggu endpoint IN sebelumnya benar2 selesai (NAKSTS clear berarti
         * siap) sebelum arm transfer baru -- polling sederhana, cukup untuk
         * command/response yang tidak streaming volume tinggi. */
        while (USBx_INEP(1)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) { }

        ep_in_transfer(1, &data[sent], chunk);

        /* Tunggu transfer complete (XFRC) paket ini sebelum lanjut ke
         * paket berikutnya -- dicek lewat DIEPINT, di-clear manual di sini
         * (bukan lewat ISR) supaya WriteBuf tetap sinkron/blocking sesuai
         * kontrak API di header. */
        while (!(USBx_INEP(1)->DIEPINT & USB_OTG_DIEPINT_XFRC)) { }
        USBx_INEP(1)->DIEPINT = USB_OTG_DIEPINT_XFRC;

        sent = (uint16_t)(sent + chunk);
    }

    /* ZLP kalau len kelipatan pas MPS (short packet marker end-of-transfer
     * di sisi host CDC) -- hanya perlu kalau len>0 dan habis pas di
     * kelipatan MPS. */
    if (len > 0u && (len % EP1_MPS) == 0u) {
        while (USBx_INEP(1)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) { }
        ep_in_transfer(1, NULL, 0);
        while (!(USBx_INEP(1)->DIEPINT & USB_OTG_DIEPINT_XFRC)) { }
        USBx_INEP(1)->DIEPINT = USB_OTG_DIEPINT_XFRC;
    }

    return 1;
}

/* ---------------------------------------------------------------------------
 * ISR utama OTG_FS
 * ------------------------------------------------------------------------- */

void OTG_FS_IRQHandler(void)
{
    uint32_t gintsts = USBx->GINTSTS;

    if (gintsts & USB_OTG_GINTSTS_USBRST) {
        USBx->GINTSTS = USB_OTG_GINTSTS_USBRST;
        s_device_configured = 0;
        s_host_dtr = 0;
        s_ctrl_stage = CTRL_STAGE_IDLE;
        ep0_out_prepare();
    }

    if (gintsts & USB_OTG_GINTSTS_ENUMDNE) {
        USBx->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
        /* Full-speed selalu MPS0=64 di desain ini -- tidak perlu baca DSTS
         * untuk cabang speed lain karena OTG_FS core hanya mendukung FS. */
        ep0_out_prepare();
    }

    if (gintsts & USB_OTG_GINTSTS_RXFLVL) {
        USBx->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM; /* mask sementara selama drain FIFO */

        uint32_t status = USBx->GRXSTSP;
        uint8_t ep_num = (uint8_t)(status & USB_OTG_GRXSTSP_EPNUM);
        uint8_t pktsts = (uint8_t)((status & USB_OTG_GRXSTSP_PKTSTS) >> USB_OTG_GRXSTSP_PKTSTS_Pos);
        uint16_t bcnt  = (uint16_t)((status & USB_OTG_GRXSTSP_BCNT) >> USB_OTG_GRXSTSP_BCNT_Pos);

        if (pktsts == 0x06u) {
            /* SETUP packet received (selalu 8 byte tepat) */
            static uint8_t setup_buf[8];
            volatile uint32_t *fifo = fifo_ptr(0);
            for (int i = 0; i < 2; i++) {
                uint32_t w = *fifo;
                memcpy(&setup_buf[i * 4], &w, 4);
            }
            handle_setup(setup_buf);
        } else if (pktsts == 0x02u) {
            /* OUT data received */
            volatile uint32_t *fifo = fifo_ptr(ep_num);
            uint16_t words = (uint16_t)((bcnt + 3u) / 4u);
            for (uint16_t i = 0; i < words; i++) {
                uint32_t w = *fifo;
                if (ep_num == 1u) {
                    /* Data CDC bulk dari host -> masuk ring buffer RX */
                    uint16_t remaining = (uint16_t)(bcnt - i * 4u);
                    uint8_t nbytes = (remaining >= 4u) ? 4u : (uint8_t)remaining;
                    for (uint8_t b = 0; b < nbytes; b++) {
                        ring_push(&s_rx_ring, (uint8_t)(w >> (b * 8u)));
                    }
                }
                /* ep_num==0 (data stage control OUT, mis. SET_LINE_CODING) --
                 * sengaja di-drain tanpa disimpan, lihat catatan di
                 * handle_setup() soal SET_LINE_CODING. */
            }
        } else if (pktsts == 0x03u || pktsts == 0x04u) {
            /* OUT transfer complete / SETUP complete -- tidak ada data,
             * hanya penanda, tidak perlu aksi tambahan di sini karena
             * re-arming endpoint dilakukan terpisah (ep0_out_prepare /
             * ep_out_prepare) setiap siklus. */
        }

        USBx->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
    }

    if (gintsts & USB_OTG_GINTSTS_OEPINT) {
        uint32_t daint = USBx_DEVICE->DAINT;
        if (daint & (1u << (16 + 0))) { /* EP0 OUT */
            uint32_t doepint = USBx_OUTEP(0)->DOEPINT;
            if (doepint & USB_OTG_DOEPINT_STUP) {
                USBx_OUTEP(0)->DOEPINT = USB_OTG_DOEPINT_STUP;
            }
            if (doepint & USB_OTG_DOEPINT_XFRC) {
                USBx_OUTEP(0)->DOEPINT = USB_OTG_DOEPINT_XFRC;
                ep0_out_prepare(); /* siap terima setup/data OUT berikutnya */
            }
        }
        if (daint & (1u << (16 + 1))) { /* EP1 OUT */
            uint32_t doepint = USBx_OUTEP(1)->DOEPINT;
            if (doepint & USB_OTG_DOEPINT_XFRC) {
                USBx_OUTEP(1)->DOEPINT = USB_OTG_DOEPINT_XFRC;
                ep_out_prepare(1, EP1_MPS); /* re-arm, siap terima paket bulk berikutnya */
            }
        }
    }

    if (gintsts & USB_OTG_GINTSTS_IEPINT) {
        uint32_t daint = USBx_DEVICE->DAINT;
        if (daint & (1u << 0)) { /* EP0 IN */
            uint32_t diepint = USBx_INEP(0)->DIEPINT;
            if (diepint & USB_OTG_DIEPINT_XFRC) {
                USBx_INEP(0)->DIEPINT = USB_OTG_DIEPINT_XFRC;

                if (s_pending_address != 0u && s_ctrl_stage == CTRL_STAGE_STATUS_IN) {
                    /* SET_ADDRESS: terapkan alamat SETELAH status stage
                     * selesai dikirim (persis sesuai urutan yang diwajibkan
                     * spec USB 9.4.6). */
                    USBx_DEVICE->DCFG = (USBx_DEVICE->DCFG & ~USB_OTG_DCFG_DAD)
                                       | ((uint32_t)s_pending_address << USB_OTG_DCFG_DAD_Pos);
                    s_pending_address = 0u;
                }

                if (s_ctrl_stage == CTRL_STAGE_DATA_IN) {
                    if (s_ctrl_tx_remaining > 0u) {
                        uint16_t chunk = (s_ctrl_tx_remaining > EP0_MPS) ? EP0_MPS : s_ctrl_tx_remaining;
                        ep_in_transfer(0, s_ctrl_tx_ptr, chunk);
                        s_ctrl_tx_ptr += chunk;
                        s_ctrl_tx_remaining = (uint16_t)(s_ctrl_tx_remaining - chunk);
                    } else {
                        /* Data stage selesai -> host akan kirim status OUT
                         * (ZLP) yang cukup di-drain otomatis oleh
                         * ep0_out_prepare(), tidak perlu aksi eksplisit. */
                        s_ctrl_stage = CTRL_STAGE_IDLE;
                    }
                } else {
                    s_ctrl_stage = CTRL_STAGE_IDLE;
                }
            }
        }
        /* EP1 IN (bulk) XFRC ditangani via polling langsung di
         * BSP_USB_CDC_WriteBuf(), tidak perlu logic tambahan di sini --
         * tapi flag XFRC tetap harus di-clear supaya interrupt tidak
         * nyangkut pending terus (di-clear di WriteBuf itu sendiri). */
    }
}
