#pragma once

#include <subsystem/IntakeSubsystems/Constants.h>

#include <rev/SparkMax.h>

#include <frc2/command/SubsystemBase.h>

class IntakeSubsystem : public frc2::SubsystemBase{
    public:
    IntakeSubsystem()

    void On();

    void Off();

    private:

    rev::spark::SparkMax Intake;



};
