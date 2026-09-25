#include "auto/AutoCommands.h"

#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/path/PathPlannerPath.h>

#include "subsystems/shooter/ShooterCommands.h"
#include "subsystems/shooter/ShooterConstants.h"

namespace {
frc2::CommandPtr StopEverything(Shooter& shooter, Injector& injector, Agitator& agitator,
                                Intake& intake) {
  return frc2::cmd::RunOnce(
      [&] {
        shooter.Stop();
        injector.Stop();
        agitator.Stop();
        intake.StopRollers();
      },
      {&shooter, &injector, &agitator, &intake});
}

// shoots from the starting spot for 10 seconds with the intake down and running out
frc2::CommandPtr ShootFromStart(Shooter& shooter, Injector& injector, Agitator& agitator,
                                Intake& intake, double rpm) {
  return frc2::cmd::Sequence(shooter.SpinUpCommand(rpm), shooter.WaitUntilAtSpeedCommand(3_s),
                             intake.EjectCommand(),
                             ShooterCommands::Feed(injector, agitator).WithTimeout(10_s),
                             StopEverything(shooter, injector, agitator, intake));
}
}  // namespace

void AutoCommands::RegisterNamedCommands(Shooter& shooter, Injector& injector, Agitator& agitator,
                                         Intake& intake) {
  using pathplanner::NamedCommands;

  // All your named commands
  /*
  Charge Hub
  Charge Trench
  Charge Corner
  Charge Tower
  Wait For Speed
  Agitator Off
  Full Power Shot
  Stop Shooting
  Intake
  Eject
  Intake Off
  Arm Up
  Arm Down
  Stop All
  */

  NamedCommands::registerCommand(
      "Charge Hub", ShooterCommands::Charge(shooter, injector, Shooter::Location::kHub));
  NamedCommands::registerCommand(
      "Charge Trench", ShooterCommands::Charge(shooter, injector, Shooter::Location::kTrench));
  NamedCommands::registerCommand(
      "Charge Corner", ShooterCommands::Charge(shooter, injector, Shooter::Location::kCorner));
  NamedCommands::registerCommand(
      "Charge Tower", ShooterCommands::Charge(shooter, injector, Shooter::Location::kTower));
  NamedCommands::registerCommand("Wait For Speed", shooter.WaitUntilAtSpeedCommand(3_s));
  NamedCommands::registerCommand("Agitator On",
                                 frc2::cmd::RunOnce([&agitator] { agitator.Feed(); }, {&agitator}));
  NamedCommands::registerCommand("Agitator Off",
                                 frc2::cmd::RunOnce([&agitator] { agitator.Stop(); }, {&agitator}));
  NamedCommands::registerCommand("Full Power Shot",
                                 ShooterCommands::StartFullPowerShot(shooter, injector, agitator));
  NamedCommands::registerCommand("Stop Shooting",
                                 ShooterCommands::StopShooting(shooter, injector, agitator));
  NamedCommands::registerCommand("Intake", intake.IntakeCommand());
  NamedCommands::registerCommand("Eject", intake.EjectCommand());
  NamedCommands::registerCommand(
      "Intake Off", frc2::cmd::RunOnce([&intake] { intake.StopRollers(); }, {&intake}));
  NamedCommands::registerCommand("Arm Up", intake.StowCommand());
  NamedCommands::registerCommand(
      "Arm Down",
      frc2::cmd::RunOnce([&intake] { intake.SetArmGoal(Intake::ArmGoal::kDeploy); }, {&intake}));
  NamedCommands::registerCommand("Stop All", StopEverything(shooter, injector, agitator, intake));
}

frc2::CommandPtr AutoCommands::ShootHub(Shooter& shooter, Injector& injector, Agitator& agitator,
                                        Intake& intake) {
  return ShootFromStart(shooter, injector, agitator, intake,
                        shooter.GetLocationRpm(Shooter::Location::kHub));
}

frc2::CommandPtr AutoCommands::ShootHubRight(Shooter& shooter, Injector& injector,
                                             Agitator& agitator, Intake& intake) {
  return ShootFromStart(shooter, injector, agitator, intake, ShooterConstants::kHubRightRpm);
}

frc2::CommandPtr AutoCommands::ShootTower(Shooter& shooter, Injector& injector, Agitator& agitator,
                                          Intake& intake) {
  return ShootFromStart(shooter, injector, agitator, intake,
                        shooter.GetLocationRpm(Shooter::Location::kTower));
}

frc2::CommandPtr AutoCommands::ShootTrench(Shooter& shooter, Injector& injector, Agitator& agitator,
                                           Intake& intake) {
  return ShootFromStart(shooter, injector, agitator, intake,
                        shooter.GetLocationRpm(Shooter::Location::kTrench));
}

frc2::CommandPtr AutoCommands::ShootCorner(Shooter& shooter, Injector& injector, Agitator& agitator,
                                           Intake& intake) {
  return ShootFromStart(shooter, injector, agitator, intake,
                        shooter.GetLocationRpm(Shooter::Location::kCorner));
}

frc2::CommandPtr AutoCommands::RightSide(Shooter& shooter, Injector& injector, Agitator& agitator,
                                         Intake& intake) {
  auto path = pathplanner::PathPlannerPath::fromPathFile("Right Side");

  return frc2::cmd::Sequence(
      shooter.SpinUpCommand(shooter.GetLocationRpm(Shooter::Location::kHub)),
      shooter.WaitUntilAtSpeedCommand(3_s),
      ShooterCommands::Feed(injector, agitator).WithTimeout(5_s),

      // spin up while driving
      shooter.SpinUpCommand(shooter.GetLocationRpm(Shooter::Location::kCorner)),
      pathplanner::AutoBuilder::resetOdom(path->getStartingDifferentialPose()),
      intake.IntakeCommand(), pathplanner::AutoBuilder::followPath(path), intake.StowCommand(),

      shooter.WaitUntilAtSpeedCommand(4_s),
      ShooterCommands::Feed(injector, agitator).WithTimeout(4_s), shooter.StopCommand());
}
