#pragma once

#include <frc/AnalogInput.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <rev/SparkFlex.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/RelativeEncoder.h>

#include <subsystems/ShooterSubsystem/Constants.h>
#include <GlobalConstants.h>

using namespace frc;

class ShooterSubsystem : public frc2::SubsystemBase {
public:
    ShooterSubsystem();

    void Periodic() override;

    // ── Fixed RPM presets ────────────────────────
    void ShooterCorner();
    void ShooterOff();
    void ShooterHubRight();
    void ShooterIntake();
    void ShooterIn();
    void ShooterBarge();
    void ShooterTower();
    void ShooterHub();
    void SetShooterRPM(double rpm);

    // ── Distance-based RPM ────────────────────────────────────
    // Pass in the distance from DriveSubsystem::GetDistanceToHub().
    // Internally interpolates the lookup table and calls SetShooterRPM().
    // Returns the RPM that was set so you can log it.
    double SetShooterRPMFromDistance(double distanceMeters);

    // ── Telemetry ─────────────────────────────────────────────
    double GetLeftRPM()  const;
    double GetRightRPM() const;
    bool   AtTargetRPM() const;

    // ── Legacy stubs ─────────────────────────────────────────
    void SetShooterPower(double newPower);
    void GetShooterPower();
    void SetShooterState(int newState);
    int  GetShooterStates();
    void SetShooterBrakeMode(bool state);
    void ConfigShooter();

private:
    rev::spark::SparkFlex                leftShooterMotor;
    rev::spark::SparkFlex                rightShooterMotor;
    rev::spark::SparkClosedLoopController leftPID;
    rev::spark::SparkClosedLoopController rightPID;
    rev::spark::SparkRelativeEncoder      leftEncoder;
    rev::spark::SparkRelativeEncoder      rightEncoder;

    int    ShooterState = ShooterConstants::ShooterStates::kShooterPowerMode;
    double targetRPM    = ShooterConstants::kStopRPM;

    // Interpolates between kRpmTable entries for a given distance
    static double InterpolateRPM(double distanceMeters);
};