#pragma once

#include <frc/AnalogInput.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <rev/SparkMax.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/RelativeEncoder.h>

#include <subsystems/IntakeSubsystem/Constants.h>
#include <GlobalConstants.h>

using namespace frc;

class IntakeSubsystem : public frc2::SubsystemBase {
public:
    IntakeSubsystem();

    void Periodic() override;

    // ── Roller control ─────────────────────────────────
    void IntakeOut();
    void IntakeOff();
    void IntakeIntake();
    void IntakeIn();
    void SetIntakePower(double newPower);
    void GetIntakePower();
    void SetIntakeState(int newState);
    int  GetIntakeStates();
    void SetIntakeBrakeMode(bool state);
    void ConfigIntake();

    // ── Arm control ─────────────────────────────────────────
    void SetArmAngle(double degrees);
    void Deploy();
    void DeployReverse();
    void Stow();
    double GetArmAngle() const;
    bool   ArmAtTarget() const;

private:
    int    IntakeState = IntakeConstants::IntakeStates::kIntakePowerMode;
    double IntakePower = IntakeConstants::kIntakeDefaultPower;

    // Roller – SparkMax
    rev::spark::SparkMax intakeMotor;

    // Arm – SparkMax + NEO
    rev::spark::SparkMax                  armMotor;
    rev::spark::SparkClosedLoopController armPID;
    rev::spark::SparkRelativeEncoder      armEncoder;

    double targetAngle_deg = IntakeConstants::kStowedAngle;
};