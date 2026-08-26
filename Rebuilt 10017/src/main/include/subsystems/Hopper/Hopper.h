#pragma once 

#include <frc2/command/SubsystemBase.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include "Constants.h"


class HopperSubsystem : public frc2::SubsystemBase {

public: 

    HopperSubsystem();
    
    void RunHopper();
    
    void StopHopper();

private: 

    frc::PWMSparkMax hopperMotor{

        HopperConstants::kHopperMotor
    };

};