#pragma once

#include <units/length.h>
#include <units/mass.h>

namespace IntakeConstants {

namespace Rollers {
inline constexpr int kMotorId = 4;
inline constexpr bool kInverted = true;

// motor 10 follows motor 4, true makes it spin the other way
inline constexpr int kSecondMotorId = 10;
inline constexpr bool kSecondOpposesFirst = false;

// eject runs the rollers backwards
inline constexpr double kIntakePercent = 100.0;
inline constexpr double kEjectPercent = 80.0;
}  // namespace Rollers

namespace Pivot {
inline constexpr int kMotorId = 3;
inline constexpr bool kInverted = false;

// 20:1, used to say 100:1
inline constexpr double kDegreesPerMotorRotation = 360.0 / 20.0;

inline constexpr double kStowedAngleDeg = 1.0;
inline constexpr double kDeployedAngleDeg = 77.0;
inline constexpr double kToleranceDeg = 2.0;
inline constexpr double kMinAngleDeg = 0.0;
inline constexpr double kMaxAngleDeg = 95.0;

inline constexpr double kP = 0.05;
inline constexpr double kI = 0.0;
inline constexpr double kD = 0.0;
inline constexpr double kMaxOutput = 0.3;
}  // namespace Pivot

namespace Sim {
// guess
inline constexpr units::meter_t kArmLength = 0.4_m;
inline constexpr units::kilogram_t kArmMass = 3_kg;
}  // namespace Sim

}  // namespace IntakeConstants
