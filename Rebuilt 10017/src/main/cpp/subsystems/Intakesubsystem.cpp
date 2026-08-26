#include "subsystems/IntakeSubsystem/IntakeSubsystem.h"
#include "rev/config/SparkMaxConfig.h"

#include <iostream>
#include <cmath>

using namespace frc;
using namespace rev::spark;

IntakeSubsystem::IntakeSubsystem()
    : intakeMotor{IntakeConstants::kIntakePort, SparkLowLevel::MotorType::kBrushless},
      armMotor   {IntakeConstants::kArmPort,    SparkLowLevel::MotorType::kBrushless},
      armPID     {armMotor.GetClosedLoopController()},
      armEncoder {armMotor.GetEncoder()}
{
    // ── Roller (SparkMax) config ─────────────────────────────
    rev::spark::SparkMaxConfig rollerConfig{};
    rollerConfig.Inverted(true)
                .SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kCoast);

    intakeMotor.Configure(rollerConfig,
        rev::ResetMode::kResetSafeParameters,
        rev::PersistMode::kPersistParameters);

    // ── Arm (SparkMax) config ─────────────────────────────────
    rev::spark::SparkMaxConfig armConfig{};
    armConfig.Inverted(false)
             .SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);

    armConfig.encoder
        .PositionConversionFactor(IntakeConstants::kArmPositionFactor)
        .VelocityConversionFactor(IntakeConstants::kArmVelocityFactor);

    armConfig.closedLoop
        .Pidf(IntakeConstants::kArmP,
              IntakeConstants::kArmI,
              IntakeConstants::kArmD,
              IntakeConstants::kArmFF)
        .OutputRange(IntakeConstants::kArmMinOutput,
                     IntakeConstants::kArmMaxOutput);

    armConfig.softLimit
        .ForwardSoftLimit(IntakeConstants::kArmMaxAngle)
        .ForwardSoftLimitEnabled(true)
        .ReverseSoftLimit(IntakeConstants::kArmMinAngle)
        .ReverseSoftLimitEnabled(true);

    armMotor.Configure(armConfig,
        rev::ResetMode::kResetSafeParameters,
        rev::PersistMode::kPersistParameters);

    armEncoder.SetPosition(0.0);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Periodic
// ─────────────────────────────────────────────────────────────────────────────
void IntakeSubsystem::Periodic()
{
    frc::SmartDashboard::PutNumber("Intake/ArmAngle_deg",  GetArmAngle());
    frc::SmartDashboard::PutNumber("Intake/ArmTarget_deg", targetAngle_deg);
    frc::SmartDashboard::PutBoolean("Intake/ArmAtTarget",  ArmAtTarget());
}

// ─────────────────────────────────────────────────────────────────────────────
//  Roller control
// ─────────────────────────────────────────────────────────────────────────────
void IntakeSubsystem::IntakeOut()    { intakeMotor.Set(-IntakePower); }
void IntakeSubsystem::IntakeOff()    { intakeMotor.Set(0.0); }
void IntakeSubsystem::IntakeIntake() { intakeMotor.Set(IntakePower); }
void IntakeSubsystem::IntakeIn()     { intakeMotor.Set(IntakePower); }

void IntakeSubsystem::SetIntakePower(double newPower) {
    IntakePower = newPower;
    intakeMotor.Set(IntakePower);
}
void IntakeSubsystem::GetIntakePower()             {}
void IntakeSubsystem::SetIntakeState(int newState) { IntakeState = newState; }
int  IntakeSubsystem::GetIntakeStates()            { return IntakeState; }
void IntakeSubsystem::SetIntakeBrakeMode(bool state) {}
void IntakeSubsystem::ConfigIntake() {}

// ─────────────────────────────────────────────────────────────────────────────
//  Arm control
// ─────────────────────────────────────────────────────────────────────────────
void IntakeSubsystem::SetArmAngle(double degrees)
{
    targetAngle_deg = degrees;
    armPID.SetReference(degrees, SparkLowLevel::ControlType::kPosition);
}

void IntakeSubsystem::Deploy()
{
    SetArmAngle(IntakeConstants::kDeployAngle);
    intakeMotor.Set(IntakePower); 
}

void IntakeSubsystem::DeployReverse()
{
    SetArmAngle(IntakeConstants::kDeployAngle);
    intakeMotor.Set(-0.8);
}

void IntakeSubsystem::Stow()
{
    SetArmAngle(IntakeConstants::kStowedAngle);  
    intakeMotor.Set(0.0);
}

double IntakeSubsystem::GetArmAngle() const
{
    return armEncoder.GetPosition();
}

bool IntakeSubsystem::ArmAtTarget() const
{
    return std::abs(GetArmAngle() - targetAngle_deg) < IntakeConstants::kAngleTolerance;
}