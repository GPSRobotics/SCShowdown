#pragma once 

#include <frc2/command/SubsystemBase.h>
#include <frc/motorcontrol/PWMSparkFlex.h> 
#include "Constants.h"


class IndexerSubsystem : public frc2::SubsystemBase {

public: 

    IndexerSubsystem();
    
    void RunIndexer();
    
    void StopIndexer();

private: 

    frc::PWMSparkFlex indexerMotor{

        IndexerConstants::kIndexerMotor
    };

};