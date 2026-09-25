#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/agitator/Agitator.h"
#include "subsystems/injector/Injector.h"
#include "subsystems/intake/Intake.h"
#include "subsystems/shooter/Shooter.h"

namespace AutoCommands {

// commands PathPlanner autos can use by name. all of them end right away except Wait For Speed
//   Charge Hub, Charge Trench, Charge Corner, Charge Tower, Wait For Speed, Agitator On,
//   Agitator Off, Full Power Shot, Stop Shooting, Intake, Eject, Intake Off, Arm Up, Arm Down,
//   Stop All
void RegisterNamedCommands(Shooter& shooter, Injector& injector, Agitator& agitator,
                           Intake& intake);

frc2::CommandPtr ShootHub(Shooter& shooter, Injector& injector, Agitator& agitator, Intake& intake);
frc2::CommandPtr ShootHubRight(Shooter& shooter, Injector& injector, Agitator& agitator,
                               Intake& intake);
frc2::CommandPtr ShootTower(Shooter& shooter, Injector& injector, Agitator& agitator,
                            Intake& intake);
frc2::CommandPtr ShootTrench(Shooter& shooter, Injector& injector, Agitator& agitator,
                             Intake& intake);
frc2::CommandPtr ShootCorner(Shooter& shooter, Injector& injector, Agitator& agitator,
                             Intake& intake);

// shoots, drives the Right Side path while intaking, then shoots again (start at the path start)
frc2::CommandPtr RightSide(Shooter& shooter, Injector& injector, Agitator& agitator,
                           Intake& intake);

}  // namespace AutoCommands
