#include "calib_gate.h"

bool CalibGate_ArmAllowed(bool primary_present,   bool primary_calibrated,
                          bool secondary_present, bool secondary_calibrated)
{
    if (!primary_present || !primary_calibrated) {
        return false;
    }
    if (secondary_present && !secondary_calibrated) {
        return false;
    }
    return true;
}