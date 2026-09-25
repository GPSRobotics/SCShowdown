#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/agitator/Agitator.h"
#include "subsystems/injector/Injector.h"
#include "subsystems/shooter/Shooter.h"

namespace ShooterCommands {

// runs agitator and injector into the shooter until interrupted
frc2::CommandPtr Feed(Injector& injector, Agitator& agitator);

// spins the shooter up for a spot and runs the injector at that spots percent. the agitator is
// what actually sends balls in
frc2::CommandPtr Charge(Shooter& shooter, Injector& injector, Shooter::Location location);

// Charge, or turns the shooter and injector off if that spot is already charged (d-pad)
frc2::CommandPtr ToggleCharge(Shooter& shooter, Injector& injector, Shooter::Location location);

// shooter at full power, then the injector at full power and the agitator on after the delay.
// ends once everything is on and leaves it running
frc2::CommandPtr StartFullPowerShot(Shooter& shooter, Injector& injector, Agitator& agitator);

// StartFullPowerShot that keeps going until canceled, then turns all three off (rt)
frc2::CommandPtr FullPowerShot(Shooter& shooter, Injector& injector, Agitator& agitator);

frc2::CommandPtr StopShooting(Shooter& shooter, Injector& injector, Agitator& agitator);

// runs agitator and injector backwards to unjam, or stops them if theyre already going
frc2::CommandPtr ToggleUnjam(Injector& injector, Agitator& agitator);

}  // namespace ShooterCommands
