/**
 * @file    scheduler.c
 * @brief   Implementasi scheduler cooperative. Lihat scheduler.h untuk
 *          dokumentasi API dan kebijakan penjadwalan.
 */

#include "scheduler.h"
#include "systick.h"

#include <stddef.h>

/* Perbandingan waktu yang aman terhadap wrap 32-bit: hitung selisih sebagai
 * unsigned lalu tafsirkan sebagai signed. Selama |selisih| < ~35 menit,
 * hasilnya benar meski counter sudah wrap di antaranya. Jangan diganti
 * dengan (now >= due) — itu akan salah persis di momen wrap. */
static inline bool time_reached(uint32_t now_us, uint32_t due_us)
{
    return (int32_t)(now_us - due_us) >= 0;
}

void Scheduler_Init(Scheduler_t *sched)
{
    if (sched == NULL) {
        return;
    }

    sched->task_count      = 0U;
    sched->loop_count      = 0U;
    sched->idle_loop_count = 0U;
    sched->initialized     = true;
}

int Scheduler_AddTask(Scheduler_t *sched, const char *name, SchedulerTaskFn fn,
                      void *ctx, uint32_t rate_hz)
{
    if (sched == NULL || !sched->initialized || fn == NULL || rate_hz == 0U) {
        return -1;
    }
    if (sched->task_count >= SCHEDULER_MAX_TASKS) {
        return -1;
    }

    uint32_t period_us = 1000000UL / rate_hz;
    if (period_us == 0U) {
        /* rate_hz > 1 MHz — tidak masuk akal, hampir pasti salah ketik. */
        return -1;
    }

    uint8_t index = sched->task_count;
    SchedulerTask_t *task = &sched->tasks[index];

    task->name          = name;
    task->fn            = fn;
    task->ctx           = ctx;
    task->period_us     = period_us;
    task->last_run_us   = 0U;
    task->max_exec_us   = 0U;
    task->overrun_count = 0U;
    task->run_count     = 0U;
    task->enabled       = true;
    task->_has_run_once = false;

    /* Sebar jadwal awal tiap task dengan offset berbeda (index * 1/4 periode)
     * supaya task-task yang laju-nya kelipatan satu sama lain — mis. 1000Hz,
     * 200Hz, 50Hz — tidak selalu jatuh tempo di mikrodetik yang sama persis.
     * Tanpa ini, tiap 20ms semua task menumpuk di satu putaran loop dan
     * menghasilkan lonjakan jitter periodik yang langsung terasa di loop
     * kontrol. */
    task->next_due_us = get_micros() + ((uint32_t)index * (period_us / 4U));

    sched->task_count++;
    return (int)index;
}

void Scheduler_SetTaskEnabled(Scheduler_t *sched, int task_index, bool enabled)
{
    if (sched == NULL || task_index < 0 || task_index >= (int)sched->task_count) {
        return;
    }

    SchedulerTask_t *task = &sched->tasks[task_index];

    if (enabled && !task->enabled) {
        /* Resync: jangan biarkan task yang baru dinyalakan mengira dirinya
         * tertinggal sejak terakhir kali aktif. */
        uint32_t now_us = get_micros();
        task->next_due_us   = now_us + task->period_us;
        task->last_run_us   = now_us;
        task->_has_run_once = false;
    }

    task->enabled = enabled;
}

void Scheduler_Tick(Scheduler_t *sched)
{
    if (sched == NULL || !sched->initialized) {
        return;
    }

    bool ran_something = false;

    for (uint8_t i = 0; i < sched->task_count; i++) {
        SchedulerTask_t *task = &sched->tasks[i];

        if (!task->enabled) {
            continue;
        }

        uint32_t now_us = get_micros();
        if (!time_reached(now_us, task->next_due_us)) {
            continue;
        }

        /* --- Hitung dt terukur untuk callback --- */
        float dt_seconds;
        if (task->_has_run_once) {
            dt_seconds = (float)(now_us - task->last_run_us) * 1.0e-6f;
        } else {
            /* Eksekusi pertama: tidak ada titik referensi, pakai periode
             * nominal. Jangan pakai 0 atau nilai liar — modul hilir
             * (PID, AHRS) akan menolak dt itu dan melewati satu siklus. */
            dt_seconds = (float)task->period_us * 1.0e-6f;
        }

        task->last_run_us   = now_us;
        task->_has_run_once = true;

        /* --- Jalankan --- */
        task->fn(task->ctx, dt_seconds);
        task->run_count++;

        /* --- Statistik waktu eksekusi --- */
        uint32_t exec_us = get_micros() - now_us;
        if (exec_us > task->max_exec_us) {
            task->max_exec_us = exec_us;
        }

        /* --- Jadwalkan eksekusi berikutnya --- */
        task->next_due_us += task->period_us;

        /* Kalau jadwal berikutnya ternyata SUDAH lewat, berarti task ini
         * tertinggal lebih dari satu periode penuh. Resync ke sekarang dan
         * catat overrun — membiarkannya mengejar akan membuat task ini
         * dieksekusi berkali-kali beruntun dan memblokir task lain. */
        uint32_t after_us = get_micros();
        if (time_reached(after_us, task->next_due_us)) {
            task->overrun_count++;
            task->next_due_us = after_us + task->period_us;
        }

        ran_something = true;
    }

    sched->loop_count++;
    if (!ran_something) {
        sched->idle_loop_count++;
    }
}

const SchedulerTask_t *Scheduler_GetTask(const Scheduler_t *sched, int task_index)
{
    if (sched == NULL || task_index < 0 || task_index >= (int)sched->task_count) {
        return NULL;
    }
    return &sched->tasks[task_index];
}

uint32_t Scheduler_GetTotalOverruns(const Scheduler_t *sched)
{
    if (sched == NULL) {
        return 0U;
    }

    uint32_t total = 0U;
    for (uint8_t i = 0; i < sched->task_count; i++) {
        total += sched->tasks[i].overrun_count;
    }
    return total;
}
