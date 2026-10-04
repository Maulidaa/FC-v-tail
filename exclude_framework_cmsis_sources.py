"""
Proyek ini punya SystemInit()/SystemCoreClock sendiri (src/system/system_clock.c)
dan vector table/Reset_Handler sendiri (src/startup/startup_stm32f411ceux.s).

Paket PlatformIO "framework-cmsis-stm32f4" ikut menyediakan dan otomatis
meng-compile system_stm32f4xx.c serta gcc/startup_stm32f411xe.s miliknya
sendiri walau kita tidak memanggilnya dari mana pun. src_filter TIDAK
berlaku untuk source di dalam package framework (hanya untuk source di
dalam src/ proyek), jadi kedua object file itu tetap ikut linking dan
menyebabkan:
    multiple definition of `g_pfnVectors`
    multiple definition of `Default_Handler`
    multiple definition of `SystemCoreClock`
    multiple definition of `SystemInit`

Solusi resmi PlatformIO: env.AddBuildMiddleware(callback, pattern) --
dipanggil untuk setiap source node sebelum di-compile; jika callback
mengembalikan None, node itu di-skip dari build sepenuhnya.
Referensi (termasuk contoh resmi utk skip file .S):
https://docs.platformio.org/en/latest/scripting/middlewares.html

CATATAN PENTING: contoh resmi PlatformIO untuk skip file assembly pakai
signature callback(env, node) -- DUA parameter -- bukan callback(node).
PlatformIO mendeteksi jumlah argumen callback (co_argcount) untuk
menentukan cara memanggilnya, jadi signature harus (env, node) di sini
supaya konsisten dan pasti terpanggil untuk file .c MAUPUN .s/.S.
"""
Import("env")

CONFLICTING_BASENAMES_LOWER = (
    "system_stm32f4xx.c",
    "startup_stm32f411xe.s",
)


def skip_conflicting_framework_sources(env, node):
    path = node.get_path().replace("\\", "/")
    path_lower = path.lower()
    is_framework_path = "framework-cmsis" in path_lower or "frameworkcmsis" in path_lower
    if is_framework_path and path_lower.endswith(CONFLICTING_BASENAMES_LOWER):
        print("[exclude_framework_cmsis_sources] Skipping duplicate-symbol file: %s" % path)
        return None
    return node


# Tanpa pattern (None/omitted) -> callback dipanggil untuk SETIAP node yang
# masuk proses build (source .c maupun .s), filter dilakukan di dalam callback.
env.AddBuildMiddleware(skip_conflicting_framework_sources)
