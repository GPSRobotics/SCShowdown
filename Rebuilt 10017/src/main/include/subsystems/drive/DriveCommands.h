#pragma once

#include <functional>

#include <frc2/command/CommandPtr.h>

#include "subsystems/drive/Drive.h"

namespace DriveCommands {

// arcade drive
frc2::CommandPtr JoystickDrive(Drive& drive, std::function<double()> forward,
                               std::function<double()> turn);

}  // namespace DriveCommands
