#include "subsystems/shooter/ShooterCommands.h"

#include <frc2/command/Commands.h>

frc2::CommandPtr ShooterCommands::Feed(Injector& injector, Agitator& agitator) {
  return frc2::cmd::StartEnd(
             [&injector, &agitator] {
               injector.Feed();
               agitator.Feed();
             },
             [&injector, &agitator] {
               injector.Stop();
               agitator.Stop();
             },
             {&injector, &agitator})
      .WithName("Feed");
}

frc2::CommandPtr ShooterCommands::Charge(Shooter& shooter, Injector& injector,
                                         Shooter::Location location) {
  return frc2::cmd::RunOnce(
      [&shooter, &injector, location] {
        shooter.SpinUpTo(location);
        injector.Charge(shooter.GetLocationInjectorPercent(location));
      },
      {&shooter, &injector});
}

frc2::CommandPtr ShooterCommands::ToggleCharge(Shooter& shooter, Injector& injector,
                                               Shooter::Location location) {
  return frc2::cmd::Either(frc2::cmd::RunOnce(
                               [&shooter, &injector] {
                                 shooter.Stop();
                                 injector.Stop();
                               },
                               {&shooter, &injector}),
                           Charge(shooter, injector, location), [&shooter, &injector, location] {
                             return shooter.IsSpinningUpTo(location) &&
                                    injector.IsGoal(Injector::Goal::kCharge);
                           });
}

frc2::CommandPtr ShooterCommands::StartFullPowerShot(Shooter& shooter, Injector& injector,
                                                     Agitator& agitator) {
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce([&shooter] { shooter.RunFullPower(); }, {&shooter}),
      // defer so it uses the delay from the tuning page at the time it starts
      frc2::cmd::Defer([&shooter] { return frc2::cmd::Wait(shooter.GetFullPowerFeedDelay()); }, {}),
      frc2::cmd::RunOnce(
          [&injector, &agitator] {
            injector.Feed();
            agitator.Feed();
          },
          {&injector, &agitator}));
}

frc2::CommandPtr ShooterCommands::FullPowerShot(Shooter& shooter, Injector& injector,
                                                Agitator& agitator) {
  return frc2::cmd::Sequence(StartFullPowerShot(shooter, injector, agitator), frc2::cmd::Idle())
      .FinallyDo([&shooter, &injector, &agitator] {
        shooter.Stop();
        injector.Stop();
        agitator.Stop();
      })
      .WithName("FullPowerShot");
}

frc2::CommandPtr ShooterCommands::StopShooting(Shooter& shooter, Injector& injector,
                                               Agitator& agitator) {
  return frc2::cmd::RunOnce(
      [&shooter, &injector, &agitator] {
        shooter.Stop();
        injector.Stop();
        agitator.Stop();
      },
      {&shooter, &injector, &agitator});
}

frc2::CommandPtr ShooterCommands::ToggleUnjam(Injector& injector, Agitator& agitator) {
  return frc2::cmd::RunOnce(
      [&injector, &agitator] {
        if (injector.IsGoal(Injector::Goal::kReverse)) {
          injector.Stop();
          agitator.Stop();
        } else {
          injector.Reverse();
          agitator.Reverse();
        }
      },
      {&injector, &agitator});
}
