#ifndef CALIB_GATE_H
#define CALIB_GATE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Kebijakan (satu-satunya tempat didefinisikan):
 *   - IMU primer WAJIB hadir DAN terkalibrasi.
 *   - IMU sekunder wajib terkalibrasi HANYA JIKA hadir. */
bool CalibGate_ArmAllowed(bool primary_present,   bool primary_calibrated,
                          bool secondary_present, bool secondary_calibrated);

#ifdef __cplusplus
}
#endif

#endif /* CALIB_GATE_H */