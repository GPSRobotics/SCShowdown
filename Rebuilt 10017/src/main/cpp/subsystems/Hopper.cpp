#include "subsystems/Hopper/Hopper.h"


HopperSubsystem::HopperSubsystem() {
}

void HopperSubsystem::RunHopper() {
    
    hopperMotor.Set(HopperConstants::kHopperSpeed);

}