#pragma once

#include <units/moment_of_inertia.h>
#include <units/time.h>

namespace ShooterConstants {

inline constexpr int kLeftMotorId = 5;
inline constexpr int kRightMotorId = 6;
inline constexpr bool kLeftInverted = false;
inline constexpr bool kRightInverted = true;

inline constexpr double kHubRpm = 4000.0;
inline constexpr double kHubRightRpm = 4700.0;
inline constexpr double kTowerRpm = 5725.0;
inline constexpr double kTrenchRpm = 6300.0;
inline constexpr double kCornerRpm = 7000.0;
inline constexpr double kToleranceRpm = 100.0;

// injector while charging at each spot, team said about 80%
inline constexpr double kHubInjectorPercent = 80.0;
inline constexpr double kTrenchInjectorPercent = 80.0;
inline constexpr double kCornerInjectorPercent = 80.0;
inline constexpr double kTowerInjectorPercent = 80.0;

// also counts as at speed if its close and not speeding up anymore (corner is above vortex max)
inline constexpr double kMaxedOutFraction = 0.8;
inline constexpr double kMaxedOutRpmPerSecond = 200.0;

// rt runs the shooter at full power, then starts the injector and agitator this long after
inline constexpr double kFullPower = 1.0;
inline constexpr units::second_t kFullPowerFeedDelay = 1_s;

inline constexpr double kP = 0.0003;
inline constexpr double kI = 0.0;
inline constexpr double kD = 0.0;
inline constexpr double kV = 0.000175;
inline constexpr double kMinOutput = -1.0;
inline constexpr double kMaxOutput = 1.0;

namespace Sim {
// sim reads kv differently than the real sparks so it needs its own
inline constexpr double kV = 12.0 / 6784.0;

// set up flywheel to spin up at like the same speed as the real one
inline constexpr units::kilogram_square_meter_t kFlywheelMoi = 0.002_kg_sq_m;
}  // namespace Sim

}  // namespace ShooterConstants
