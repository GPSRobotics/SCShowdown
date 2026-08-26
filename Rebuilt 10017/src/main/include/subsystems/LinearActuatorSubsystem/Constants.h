//DOESN'T WORK





//DOESN'T WORK

#pragma once

#include <units/time.h>

namespace ActuonixL16 {

    // ── PWM channel wired on the roboRIO ──────────────────────
    constexpr int kPwmChannel = 0;

    // ── PWM pulse widths – typed as units::microsecond_t ─────
    constexpr units::microsecond_t kPulseMax_us      = 2000_us; // full extension
    constexpr units::microsecond_t kPulseMin_us      = 1000_us; // full retraction
    constexpr units::microsecond_t kPulseCentre_us   = 1500_us; // mid-point
    constexpr units::microsecond_t kDeadbandHigh_us  = 1520_us; // upper dead-band
    constexpr units::microsecond_t kDeadbandLow_us   = 1480_us; // lower dead-band

    // ── Physical limits ───────────────────────────────────────
    constexpr double kStroke_mm           = 100.0; // total travel (mm)
    constexpr double kMinPosition_mm      =   0.0; // fully retracted
    constexpr double kMaxPosition_mm      = 100.0; // fully extended

    // ── Safety / tuning ───────────────────────────────────────
    constexpr double kPositionTolerance_mm = 2.0;  // "close enough" band

} // namespace ActuonixL16