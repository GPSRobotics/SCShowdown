#include "subsystems/AgitatorSubsystem/AgitatorSubsystem.h"
#include "rev/config/SparkMaxConfig.h"

#include <iostream>
#include <cmath>

using namespace frc;
using namespace rev::spark;

AgitatorSubsystem::AgitatorSubsystem() :
    agitatorMotor{AgitatorConstants::kAgitatorPort, SparkLowLevel::MotorType::kBrushless}
{
    //  motor config (not inverted)
    rev::spark::SparkMaxConfig Config{};
    Config.Inverted(true)
              .SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kCoast);
    agitatorMotor.Configure(Config,
        SparkMax::ResetMode::kResetSafeParameters,
        SparkMax::PersistMode::kPersistParameters);
    }

void AgitatorSubsystem::Periodic() {}

void AgitatorSubsystem::AgitatorOut() {
    agitatorMotor.Set(-0.7);
}

void AgitatorSubsystem::AgitatorOff() {
    agitatorMotor.Set(0.0);
}

void AgitatorSubsystem::AgitatorAgitate() {
    agitatorMotor.Set(0.7);
}

void AgitatorSubsystem::AgitatorIn() {
    agitatorMotor.Set(0.7);
}

void AgitatorSubsystem::SetAgitatorPower(double newPower) {}
void AgitatorSubsystem::GetAgitatorPower() {}
void AgitatorSubsystem::SetAgitatorState(int newState) {}
int  AgitatorSubsystem::GetAgitatorStates() { return 0; }
void AgitatorSubsystem::SetAgitatorBrakeMode(bool state) {}
void AgitatorSubsystem::ConfigAgitator() {}