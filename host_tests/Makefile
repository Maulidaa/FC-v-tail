# host_tests/Makefile
#
# Test host-side (gcc biasa, tanpa toolchain ARM). Jalankan: make -C host_tests
# TIDAK dijalankan oleh PlatformIO (folder ini sengaja bukan "test/").
#
# Menggantikan Makefile ad-hoc sebelumnya (lihat catatan lama di
# test_ibus_parser.c yang sudah dihapus, digabung ke test_rx_ibus.c) --
# sekarang mengikuti konvensi host_tests/ "sungguhan" milik project
# (test_common.h gaya CHECK/TEST_SUMMARY), supaya tidak ada dua konvensi
# test berbeda berdampingan.
#
# rx_ibus.c sengaja tidak bergantung ke register MCU/CMSIS sama sekali
# (murni stdint/stdbool/string.h), jadi bisa dikompilasi native di host
# tanpa stub HAL apa pun. Hal yang sama berlaku untuk armed_state.c
# (menerima snapshot ArmedStateInputs_t, tidak meng-include modul lain)
# dan calib_gate.c (fungsi murni, tanpa dependensi apa pun) -- keduanya
# memang dirancang begitu supaya gerbang arming bisa diuji tanpa board.
#
# SAN=1 menyalakan AddressSanitizer + UBSan:  make -C host_tests SAN=1

CC      ?= gcc
S       := ../src
CFLAGS  := -std=gnu11 -O1 -g -Wall -Wextra -Wno-unused-parameter -I. \
           -I$(S)/rx -I$(S)/system -I$(S)/calibration -I$(S)/control \
           -I$(S)/fusion -I$(S)/output
LDLIBS  := -lm
B       := build

ifeq ($(SAN),1)
CFLAGS  += -fsanitize=address,undefined -fno-omit-frame-pointer
LDFLAGS += -fsanitize=address,undefined
endif

TESTS := test_rx_ibus test_armed_state test_calib_gate test_pid test_ahrs_fusion test_compass test_rth_bank test_get_status test_actuator_test test_calib_commands test_telemetry

all: $(addprefix $(B)/,$(TESTS))
	@fail=0; for t in $(TESTS); do ./$(B)/$$t || fail=1; done; exit $$fail

$(B):
	@mkdir -p $(B)

$(B)/test_rx_ibus: test_rx_ibus.c $(S)/rx/rx_ibus.c | $(B)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)

$(B)/test_armed_state: test_armed_state.c $(S)/system/armed_state.c | $(B)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)

$(B)/test_calib_gate: test_calib_gate.c $(S)/calibration/calib_gate.c | $(B)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)

$(B)/test_pid: test_pid.c $(S)/control/pid.c | $(B)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)

$(B)/test_ahrs_fusion: test_ahrs_fusion.c $(S)/fusion/ahrs_fusion.c | $(B)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)

# settings.h di baris -I. (folder ini) SENGAJA menang atas settings.h
# ASLI (src/settings/, tidak ada di CFLAGS include path sama sekali) --
# compass.c include "settings.h" untuk Compass_RegisterSettings(), dan
# versi asli menyeret protocol.h -> stm32f4xx.h (CMSIS, tidak bisa
# dikompilasi native di host). Lihat settings.h & settings_stub.c di
# folder ini untuk detail lengkap kenapa dan apa risikonya (interface
# bisa diam-diam usang kalau settings.h ASLI berubah).
$(B)/test_compass: test_compass.c $(S)/fusion/compass.c $(S)/calibration/calib_mag.c settings_stub.c | $(B)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)

# rth_bank_from_heading() adalah fungsi `static` di src/main.c -- tidak
# bisa di-link langsung (main.c menyeret header MCU/CMSIS, sama alasan
# settings_stub.c dipakai di atas untuk test_compass). test_rth_bank.c
# menyalin fungsinya verbatim, jadi tidak butuh source file src/ lain.
$(B)/test_rth_bank: test_rth_bank.c | $(B)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)

# handle_get_status() adalah fungsi `static` di src/comms/command_handler.c.
# Beda dari test_compass (settings.h bisa dioverride lewat -I. karena
# src/fusion/ tidak py settings.h sendiri), command_handler.h meng-include
# "protocol.h" DARI DALAM src/comms/ -- quote-include akan selalu
# menemukan src/comms/protocol.h ASLI (-> stm32f4xx.h/CMSIS) lebih dulu,
# apa pun urutan -I. Jadi command_handler.c tidak bisa dikompilasi native
# di host sama sekali (bukan cuma linknya, sama seperti main.c untuk
# rth_bank_from_heading di atas). test_get_status.c menyalin fungsinya
# verbatim + stand-in lokal untuk semua dependency-nya, tidak butuh
# source file src/ lain.
$(B)/test_get_status: test_get_status.c | $(B)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)

clean:
	rm -rf $(B)

.PHONY: all clean

# test_actuator_test meng-link kode ASLI output_map.c + command_handler.c +
# error.c (bukan salinan). protocol.h meng-include stm32f4xx.h, jadi
# target ini butuh CMSIS include path + -DSTM32F411xE (pakai -isystem
# supaya warning bawaan CMSIS di host 64-bit tidak ikut muncul). Tidak ada
# register/peripheral yang dieksekusi -- semua batas hardware di-stub di
# test_actuator_test.c.
$(B)/test_actuator_test: CFLAGS += -DSTM32F411xE -I$(S)/comms \
    -isystem ../lib/CMSIS/Include -isystem ../lib/CMSIS/Device/ST/STM32F4xx/Include
$(B)/test_actuator_test: test_actuator_test.c $(S)/output/output_map.c \
    $(S)/comms/command_handler.c $(S)/system/error.c | $(B)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)

# test_calib_commands: kode ASLI calib_dispatcher.c + calib_accel_gyro.c +
# calib_mag.c + command_handler.c + error.c; hanya transport & hook
# main.c (Armed_IsArmed, CalibRecover_IsActive) yang di-stub. Flag CMSIS
# sama seperti test_actuator_test.
$(B)/test_calib_commands: CFLAGS += -DSTM32F411xE -I$(S)/comms \
    -isystem ../lib/CMSIS/Include -isystem ../lib/CMSIS/Device/ST/STM32F4xx/Include
$(B)/test_calib_commands: test_calib_commands.c $(S)/calibration/calib_dispatcher.c \
    $(S)/calibration/calib_accel_gyro.c $(S)/calibration/calib_mag.c \
    $(S)/comms/command_handler.c $(S)/system/error.c | $(B)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)

# test_telemetry: telemetry.c + command_handler.c + error.c ASLI; hanya
# transport yang di-stub. navigation.h (NAV_GPS_FIX_*) butuh -I nav.
$(B)/test_telemetry: CFLAGS += -DSTM32F411xE -I$(S)/comms -I$(S)/nav \
    -isystem ../lib/CMSIS/Include -isystem ../lib/CMSIS/Device/ST/STM32F4xx/Include
$(B)/test_telemetry: test_telemetry.c $(S)/comms/telemetry.c \
    $(S)/comms/command_handler.c $(S)/system/error.c | $(B)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)
