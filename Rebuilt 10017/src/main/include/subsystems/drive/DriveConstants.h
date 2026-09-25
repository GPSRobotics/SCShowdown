#pragma once

#include <numbers>

#include <frc/geometry/Pose2d.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>

namespace DriveConstants {

inline constexpr int kLeftMotorId = 2;
inline constexpr int kRightMotorId = 1;
inline constexpr bool kLeftInverted = true;
inline constexpr bool kRightInverted = false;

inline constexpr units::meter_t kTrackWidth = 0.5588_m;
inline constexpr double kWheelDiameterMeters = 0.1016;
inline constexpr double kGearRatio = 8.46;
inline constexpr double kMetersPerMotorRotation =
    kWheelDiameterMeters * std::numbers::pi / kGearRatio;

// probably a guess
inline constexpr double kMaxSpeedMetersPerSecond = 4.5;

// where odometry starts, hardcoded since theres no cameras to find the robot
inline constexpr frc::Pose2d kStartPose{3.44_m, 4.11_m, 0_deg};

namespace JoystickDrive {
inline constexpr double kDeadband = 0.1;
inline constexpr double kTurnScale = 0.5;
// slow and turn only modes multiply the turn by this too
inline constexpr double kSlowDriveScale = 0.5;
inline constexpr double kSlowTurnScale = 0.5;
}  // namespace JoystickDrive

namespace Sim {
// from pathplanner settings
inline constexpr units::kilogram_t kMass = 45.81_kg;
inline constexpr units::kilogram_square_meter_t kMoi = 6.883_kg_sq_m;
inline constexpr double kScrubFactor = 1.5;
}  // namespace Sim

}  // namespace DriveConstants
