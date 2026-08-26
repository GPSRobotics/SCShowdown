#include "subsystems/InjectorSubsystem/InjectorSubsystem.h"
#include "rev/config/SparkFlexConfig.h"

#include <iostream>
#include <cmath>

using namespace frc;
using namespace rev::spark;

InjectorSubsystem::InjectorSubsystem() :
    injectorMotor{InjectorConstants::kInjectorPort, SparkLowLevel::MotorType::kBrushless}
{
    //  motor config (not inverted)
    rev::spark::SparkFlexConfig Config{};
    Config.Inverted(true)
              .SetIdleMode(rev::spark::SparkFlexConfig::IdleMode::kCoast);
    injectorMotor.Configure(Config,
        SparkFlex::ResetMode::kResetSafeParameters,
        SparkFlex::PersistMode::kPersistParameters);
    }

void InjectorSubsystem::Periodic() {}

void InjectorSubsystem::InjectorOut() {
    injectorMotor.Set(-1.0);
}

void InjectorSubsystem::InjectorOff() {
    injectorMotor.Set(0.0);
}

void InjectorSubsystem::InjectorInjector() {
    injectorMotor.Set(1.0);
}

void InjectorSubsystem::InjectorIn() {
    injectorMotor.Set(1.0);
}

void InjectorSubsystem::SetInjectorPower(double newPower) {}
void InjectorSubsystem::GetInjectorPower() {}
void InjectorSubsystem::SetInjectorState(int newState) {}
int  InjectorSubsystem::GetInjectorStates() { return 0; }
void InjectorSubsystem::SetInjectorBrakeMode(bool state) {}
void InjectorSubsystem::ConfigInjector() {}