#pragma once

#include <frc/AnalogInput.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/SparkFlex.h>

#include <subsystems/InjectorSubsystem/Constants.h>
#include <GlobalConstants.h>

using namespace frc;

class InjectorSubsystem : public frc2::SubsystemBase {
public:
    InjectorSubsystem();

    void Periodic() override;

    void InjectorOut();
    void InjectorOff();
    void InjectorInjector();
    void InjectorIn();
    void SetInjectorPower(double newPower);
    void GetInjectorPower();
    void SetInjectorState(int newState);
    int  GetInjectorStates();
    void SetInjectorBrakeMode(bool state);
    void ConfigInjector();

private:
    int    InjectorState = InjectorConstants::InjectorStates::kInjectorPowerMode;
    double InjectorPower = InjectorConstants::kInjectorDefaultPower;

    rev::spark::SparkFlex injectorMotor;
};