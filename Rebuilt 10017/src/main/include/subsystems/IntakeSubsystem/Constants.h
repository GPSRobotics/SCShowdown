#pragma once

#include <units/time.h>
#include <units/angle.h>

namespace IntakeConstants {

    // ── CAN Ports ─────────────────────────────────────────────
    constexpr int kIntakePort   = 4;   // SparkMax (not Flex?) (NEO Vortex) - roller
    constexpr int kArmPort      = 3;   // SparkMax  (NEO)        - arm angle

    // ── Roller ────────────────────────────────────────────────
    constexpr double kIntakeDefaultPower = 1.0;
    constexpr double kHoldingPower       = -0.2;

    // ── Arm Positions (degrees) ───────────────────────────────
    constexpr double kStowedAngle    =  1.0;  // arm up / stowed
    constexpr double kDeployAngle    = 77.0;  // arm down / intaking
    constexpr double kAngleTolerance =  2.0;  // degrees "close enough"

    // ── Arm Soft Limits ───────────────────────────────────────
    constexpr float kArmMinAngle = 0.0f;
    constexpr float kArmMaxAngle = 95.0f;

    // ── Arm Encoder Conversion ────────────────────────────────
    constexpr double kArmPositionFactor = 360.0 / 100.0;
    constexpr double kArmVelocityFactor = kArmPositionFactor / 60.0;

    // ── Arm PID (SparkMax onboard, slot 0) ────────────────────
    constexpr double kArmP         =  0.05;
    constexpr double kArmI         =  0.0;
    constexpr double kArmD         =  0.0;
    constexpr double kArmFF        =  0.0;
    constexpr double kArmMinOutput = -0.3;
    constexpr double kArmMaxOutput =  0.3;

   // bool isDeploy = 0;

    // ── Intake States ─────────────────────────────────────────
    enum IntakeStates {
        kIntakeOff,
        kIntakePowerMode
    };
}