#include "subsystems/InjectorSubsystem/InjectorSubsystem.h"
#include "rev/config/SparkFlexConfig.h"

#include <iostream>
#include <cmath>

using namespace frc;
using namespace rev::spark;

InjectorSubsystem::InjectorSubsystem() :
    injectorMotor{InjectorConstants::kInjectorPort, SparkLowLevel::MotorType::kBrushless}
{
    //  Motor config (inverted)
    rev::spark::SparkFlexConfig Config{};
    Config.Inverted(true)
              .SetIdleMode(rev::spark::SparkFlexConfig::IdleMode::kCoast);
    injectorMotor.Configure(Config,
        SparkFlex::ResetMode::kResetSafeParameters,
        SparkFlex::PersistMode::kPersistParameters);
    }

void InjectorSubsystem::Periodic() {}

//Technically it would make more sense for injectorin to effectively shoot so I'll fix it later, injectorout leads to ball shooting
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