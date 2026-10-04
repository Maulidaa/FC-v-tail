# flightController (PlatformIO)

Firmware STM32F411CEU6 (bare-metal, CMSIS-only — tanpa HAL/LL) yang sudah
direstrukturisasi dari proyek STM32CubeIDE ke struktur standar PlatformIO.

## Struktur direktori

```
.
├── platformio.ini          # Konfigurasi environment, defines, linker script
├── src/                    # Seluruh source (.c) — persis isi Src/ proyek asli
│   ├── main.c
│   ├── syscalls.c
│   ├── sysmem.c
│   ├── startup/
│   │   └── startup_stm32f411ceux.s   # Vector table + reset handler (vendor)
│   ├── blackbox/  bsp/  calibration/  comms/  control/  display/
│   ├── fusion/    nav/  output/       power/  rx/       scheduler/
│   ├── sensors/   settings/           system/
├── include/                 # Header publik proyek (kosong — sesuai Inc/ asli)
├── lib/
│   └── CMSIS/               # Header ARM CMSIS + device ST STM32F4xx
│       ├── library.json
│       ├── Include/                          (CMSIS core: core_cm4.h, dst.)
│       └── Device/ST/STM32F4xx/Include/       (stm32f4xx.h, stm32f411xe.h, dst.)
└── ldscript/
    ├── STM32F411CEUX_FLASH.ld
    └── STM32F411CEUX_RAM.ld
```

## Catatan penting

- **Tidak ada HAL/LL.** Header di `lib/CMSIS` hanya berisi `Include/`
  (register-level CMSIS), bukan `Drivers/STM32F4xx_HAL_Driver`. Semua akses
  peripheral di `src/bsp/` langsung ke register.
- **`Device/ST/STM32F4xx/Source/Templates` sengaja TIDAK disalin.**
  `SystemInit()` dan `SystemCoreClock` proyek ini diimplementasikan sendiri
  di `src/system/system_clock.c` — menyalin `system_stm32f4xx.c` bawaan
  CMSIS akan menyebabkan duplicate symbol saat linking.
- **Board**: `blackpill_f411ce` dipakai sebagai board definition PlatformIO
  terdekat untuk STM32F411CEU6 (flash 512KB, RAM 128KB) hanya untuk metadata
  chip/flash tool — linker script tetap pakai punya proyek sendiri
  (`ldscript/STM32F411CEUX_FLASH.ld`), bukan bawaan board.
- **Framework**: `cmsis` (bukan `stm32cube`), karena proyek tidak memakai HAL.

## Build

```
pio run
```

## Upload (ST-Link)

```
pio run -t upload
```

## Masalah yang sudah diperbaiki setelah percobaan build pertama

1. **`fatal error: xxx.h: No such file or directory`** (`error.h`,
   `bsp_pinmap.h`, `output_map.h`, `bsp_i2c.h`, `system_clock.h`, dst.)
   PlatformIO's Library Dependency Finder (LDF) hanya menambahkan folder
   berisi `.c` yang sedang dikompilasi sebagai include path — bukan seluruh
   pohon `src/`. Jadi file di `src/comms/` tidak otomatis menemukan header
   di `src/system/` atau `src/output/`. **Solusi**: setiap subfolder `src/`
   ditambahkan eksplisit sebagai `-I` di `build_flags` pada `platformio.ini`.

2. **Duplicate symbol dari `framework = cmsis`** (percobaan build #2):
   paket `framework-cmsis-stm32f4` bawaan PlatformIO otomatis meng-compile
   `system_stm32f4xx.c` dan `gcc/startup_stm32f411xe.s` miliknya sendiri —
   bentrok dengan `src/system/system_clock.c` (`SystemInit`/`SystemCoreClock`)
   dan `src/startup/startup_stm32f411ceux.s` (`g_pfnVectors`/`Default_Handler`).
   `src_filter` **tidak berlaku** untuk source di dalam package framework
   (hanya untuk `src/` milik proyek), jadi solusinya pakai
   `extra_scripts = pre:exclude_framework_cmsis_sources.py` yang memanggil
   API resmi PlatformIO `env.AddBuildMiddleware(callback, pattern)` —
   `callback` mengembalikan `None` untuk kedua file itu supaya benar-benar
   di-skip dari proses compile & link.

3. **`ld.exe: cannot find -lD:\...\STM32F411CEUX_FLASH.ld`** (percobaan
   build #2): baris `-T ldscript/....ld` sempat ditaruh di `build_flags`,
   padahal `build_flags` juga dipakai untuk tahap *compile* (bukan cuma
   *link*) — GCC compiler-driver salah mem-parsing itu sebagai
   `-l<path>` (link library bernama file itu). **Solusi**: baris `-T`
   dihapus dari `build_flags`; linker script sudah cukup lewat
   `board_build.ldscript` di bagian bawah `platformio.ini`.

4. **`multiple definition of g_pfnVectors/Default_Handler` masih muncul
   untuk file `.s`** (percobaan build #3), padahal `system_stm32f4xx.c`
   sudah berhasil di-skip. Penyebabnya: dokumentasi resmi PlatformIO untuk
   men-skip file assembly memakai signature callback
   **`callback(env, node)`** (dua parameter), bukan `callback(node)` saja
   — PlatformIO mengecek jumlah argumen callback untuk menentukan cara
   memanggilnya, dan versi builder yang dipakai di sini hanya benar-benar
   memfilter file `.s`/`.S` lewat signature dua-parameter itu. **Solusi**:
   signature callback di `exclude_framework_cmsis_sources.py` diubah jadi
   `skip_conflicting_framework_sources(env, node)`.

5. **`ldscript/...FLASH.ld:89: non constant or forward reference address
   expression for section .ARM.extab`** (percobaan build #3): linker
   script asli dari STM32CubeIDE memakai keyword `(READONLY)` pada section
   `.ARM.extab`, `.ARM`, `.preinit_array`, `.init_array`, `.fini_array`.
   Keyword itu baru didukung GCC 11+ (bahkan sudah ada komentar soal ini
   di file aslinya) — sedangkan `toolchain-gccarmnoneeabi` yang dipakai
   PlatformIO adalah **GCC 7.2.1**. **Solusi**: `(READONLY)` dihapus dari
   kelima section itu di kedua linker script (`STM32F411CEUX_FLASH.ld`
   dan `STM32F411CEUX_RAM.ld`).

6. **Upload gagal: `Debug adapter doesn't support 'hla_swd' transport`**
   (build sukses, hanya upload yang gagal). Penyebabnya: **WeAct BlackPill
   F411CE tidak punya chip ST-Link onboard** — menghubungkan board lewat
   kabel USB langsung ke port USB di board itu jalur **DFU** (USB Device
   Firmware Upgrade, bootloader bawaan ROM STM32), bukan jalur ST-Link/SWD.
   `upload_protocol = stlink` di `platformio.ini` sebelumnya salah untuk
   cara sambung ini. **Solusi**: `upload_protocol` diubah menjadi `dfu`.
   Sebelum menjalankan `pio run -t upload`, board harus dimasukkan ke DFU
   mode secara manual: tahan tombol **BOOT0**, tekan-lepas **RESET** (atau
   cabut-colok ulang USB) sambil BOOT0 masih ditahan, baru lepas BOOT0.
   Kalau nanti ingin debug/upload lewat probe ST-Link eksternal yang
   disambung ke pin SWDIO/SWCLK (bukan port USB board), baru
   `upload_protocol`/`debug_tool = stlink` relevan lagi.

## Debug tambahan: kedipan LED status (PC13) untuk diagnosa boot

`src/main.c` sudah ditambahkan/diperbaiki **dua** mekanisme debug lewat LED
onboard PC13 (bare-metal, tidak butuh debugger/UART):

### 1. `boot_fail_blink(reason)` (diperbaiki -- SEBELUMNYA TIDAK PERNAH MENYALAKAN LED)

Dipanggil kalau `SystemClock_Config()` gagal. **Sebelum perbaikan ini,
fungsi ini cuma busy-wait kosong tanpa pernah menyentuh GPIO sama sekali**
-- board akan terlihat "mati total" (LED tidak merespons apa pun), padahal
firmware sebenarnya masih berjalan, stuck di titik ini. **Ini terkonfirmasi
sebagai akar masalah** setelah build dengan perbaikan pertama menunjukkan
LED kedip cepat terus-menerus tanpa henti.

Fungsi ini sekarang menerima kode error dari `SystemClock_Config()` dan
membedakan pola kedipan sesuai titik kegagalan -- supaya tahu persis di
tahap mana clock config gagal, bukan cuma "gagal" secara umum:

- **Kedip 1x + jeda panjang, berulang terus** -> `SYSTEM_CLOCK_ERR_HSE_TIMEOUT`:
  kristal HSE 25 MHz tidak terdeteksi ready dalam batas waktu. Kemungkinan
  penyebab: kristal onboard tidak terpasang/rusak, atau board Anda
  punya kristal dengan frekuensi berbeda dari 25 MHz yang diasumsikan
  firmware ini (lihat `HSE_FREQ_HZ` di `src/system/system_clock.c`).
- **Kedip 2x + jeda panjang, berulang terus** -> `SYSTEM_CLOCK_ERR_PLL_TIMEOUT`:
  HSE sendiri OK, tapi salah satu dari (VOS ready / flash latency verify /
  PLL lock / switch ke PLL) gagal -- lebih jarang terjadi, tapi kalau
  ini yang muncul, kemungkinan penyebabnya bukan kristal, melainkan
  parameter PLL (`PLL_M`/`PLL_N`/`PLL_P`/`PLL_Q` di `system_clock.c`)
  yang tidak cocok untuk clock tree board Anda.

Fungsi ini dipanggil **sebelum** `BSP_PinMap_Init()`, jadi ia
menginisialisasi clock GPIOC & konfigurasi pin PC13 sendiri secara manual.

### 2. `debug_blink_sensor_status()` (ditambahkan sebelumnya)

Dipanggil di akhir Fase 4 `main()` (setelah semua `*_Init()` sensor
selesai) -- **hanya tercapai kalau `SystemClock_Config()` berhasil**.

- **Semua sensor OK** -> LED nyala solid ~1 detik lalu mati (SEKALI, lalu
  lanjut boot normal -- beda dari pola boot_fail_blink yang tak berhenti).
- **Ada yang gagal** -> kedip cepat N kali per sensor gagal, jeda panjang
  di antaranya:
  - N=1 -> IMU primary (MPU6500)     N=2 -> IMU secondary (MPU6050)
  - N=3 -> Baro (BMP280)             N=4 -> **OLED (SSD1306)**
  - N=5 -> Magnetometer (HMC5883/QMC5883)

### Cara baca hasilnya

- **LED sama sekali tidak merespons**: sebelum perbaikan ini, itu
  konsisten dengan `SystemClock_Config()` gagal DAN `boot_fail_blink()`
  yang lama tidak menyalakan LED.
- **Kedip 1x berulang** -> HSE timeout (lihat poin 1 di atas) -- **ini
  pola yang terkonfirmasi muncul**, jadi masalahnya ada di deteksi HSE.
- **Kedip 2x berulang** -> PLL/VOS/flash-latency timeout (HSE OK, tahap
  sesudahnya gagal).
- **LED kedip lalu berhenti (pola N-kali + jeda, satu putaran saja)** ->
  clock OK, tapi ada sensor gagal init -- hitung jumlah kedipan sesuai
  daftar `debug_blink_sensor_status()` di atas.
- **LED nyala solid sebentar lalu mati, lalu diam** -> semua sensor OK,
  termasuk OLED; masalah OLED (kalau masih ada) ada di tempat lain.

## Status saat ini: HSE timeout terkonfirmasi

Pola LED yang dilaporkan (**kedip cepat terus-menerus tanpa henti,
sebelum diferensiasi 1x/2x ditambahkan**) sudah cukup untuk memastikan
`SystemClock_Config()` gagal karena `SYSTEM_CLOCK_ERR_HSE_TIMEOUT` --
build ulang dengan kode di zip ini untuk konfirmasi polanya memang 1x
(bukan 2x). Kemungkinan penyebab HSE gagal ready, dari yang paling
mungkin:

1. **Kristal onboard tidak terpasang / lepas solderan.** Board clone
   murah kadang punya cold-joint di kristal atau kapasitor beban
   (load capacitor) -- cek fisik dengan kaca pembesar kalau memungkinkan.
2. **Board Anda bukan varian dengan kristal 25 MHz.** Sebagian varian
   BlackPill/clone STM32F411 memakai kristal 8 MHz. Kalau begitu,
   `HSE_FREQ_HZ` dan seluruh parameter PLL (`PLL_M`/`PLL_N`/`PLL_P`/
   `PLL_Q`) di `src/system/system_clock.c` perlu dihitung ulang untuk
   frekuensi kristal yang benar.
3. **`CLOCK_TIMEOUT_LOOPS` terlalu pendek** untuk kondisi tertentu --
   kurang mungkin (500000 iterasi jauh di atas waktu wajar startup HSE),
   tapi bisa dicoba dinaikkan sebagai eliminasi.

Konfirmasi paling mudah untuk poin 2: cek fisik angka yang tercetak di
badan kristal onboard board Anda (biasanya komponen metal kecil 2-3mm,
tercetak "25.000" atau "8.000" dst.).

## Yang belum divalidasi

CLI PlatformIO tidak tersedia di lingkungan tempat konversi ini dibuat,
jadi perbaikan di atas didasarkan pada log error build & upload Anda,
bukan hasil `pio run` langsung di sini. **Build sudah dikonfirmasi sukses**
oleh Anda (Flash 5.9%, RAM 2.4% terpakai); perbaikan `upload_protocol = dfu`
di atas belum divalidasi ulang — coba build+upload lagi setelah board
dimasukkan DFU mode, dan kirim log kalau masih gagal.
