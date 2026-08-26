#pragma once

#include <frc/AnalogInput.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/SparkMax.h>

#include <subsystems/AgitatorSubsystem/Constants.h>
#include <GlobalConstants.h>

using namespace frc;

class AgitatorSubsystem : public frc2::SubsystemBase {
public:
    AgitatorSubsystem();

    void Periodic() override;

    void AgitatorOut();
    void AgitatorOff();
    void AgitatorAgitate();
    void AgitatorIn();
    void SetAgitatorPower(double newPower);
    void GetAgitatorPower();
    void SetAgitatorState(int newState);
    int  GetAgitatorStates();
    void SetAgitatorBrakeMode(bool state);
    void ConfigAgitator();

private:
    int    AgitatorState = AgitatorConstants::AgitatorStates::kAgitatorPowerMode;
    double AgitatorPower = AgitatorConstants::kAgitatorDefaultPower;

    rev::spark::SparkMax agitatorMotor;
};