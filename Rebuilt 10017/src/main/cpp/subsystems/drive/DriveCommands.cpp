#include "subsystems/drive/DriveCommands.h"

#include <cmath>
#include <utility>

#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>

#include "subsystems/drive/DriveConstants.h"

using namespace DriveConstants::JoystickDrive;

namespace {
// squaring this makes it where the driving should be much nicer
double Square(double value) { return std::copysign(value * value, value); }
}  // namespace

frc2::CommandPtr DriveCommands::JoystickDrive(Drive& drive, std::function<double()> forward,
                                              std::function<double()> turn) {
  return frc2::cmd::Run(
             [&drive, forward = std::move(forward), turn = std::move(turn)] {
               // stops the robot from moving evily in auto
               if (!frc::DriverStation::IsTeleopEnabled()) {
                 drive.ArcadeDrive(0.0, 0.0);
                 return;
               }

               double forwardPower =
                   Square(frc::ApplyDeadband(forward(), kDeadband)) * drive.GetDriveScale();
               double turnPower =
                   Square(frc::ApplyDeadband(turn(), kDeadband)) * drive.GetTurnScale();
               drive.ArcadeDrive(forwardPower, turnPower);
             },
             {&drive})
      .WithName("JoystickDrive");
}
