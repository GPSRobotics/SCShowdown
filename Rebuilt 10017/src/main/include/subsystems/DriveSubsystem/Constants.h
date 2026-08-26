#pragma once

#include <units/length.h>
#include <units/angle.h>
#include <units/velocity.h>

namespace DriveConstants {

    // ── CAN IDs ────────────────────────────────────────────────
    constexpr int kRightMotorID  = 1;
    constexpr int kLeftMotorID   = 2;
    constexpr int kMiddleMotorID = 9;

    // ── Drivetrain physical dimensions ─────────────────────────
    constexpr units::meter_t kTrackWidth               = 0.5588_m;   
    constexpr double kWheelDiameterMeters              = 0.1016;
    constexpr double kWheelCircumferenceMeters         = kWheelDiameterMeters * 3.14159265;
    constexpr double kDriveGearRatio                   = 8.46;    
    constexpr double kMetersPerRotation                = kWheelCircumferenceMeters / kDriveGearRatio;

    // Top speed of the drivetrain — PathPlanner uses this to scale outputs
    // Measure by driving full power in a straight line and reading encoder velocity
    constexpr double kMaxSpeedMetersPerSecond          = 4.5;     

    // ── Odometry Reset Position ────────────────────────────────
    constexpr double kResetX_m       = 3.44;
    constexpr double kResetY_m       = 4.11;
    constexpr double kResetAngle_deg = 0.0;

    // ── 2026 REBUILT Hub Positions ─────────────────────────────
    constexpr double kBlueHubX_m = 2.90;
    constexpr double kBlueHubY_m = 4.11;
    constexpr double kRedHubX_m  = 13.64;
    constexpr double kRedHubY_m  = 4.11;
}