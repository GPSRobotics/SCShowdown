// Sam RobotContainer.h

#pragma once

#include <memory>
#include <optional>
#include <string>

#include <frc2/command/CommandPtr.h>

#include "auto/AutoChooser.h"
#include "controls/Controls.h"
#include "subsystems/agitator/Agitator.h"
#include "subsystems/drive/Drive.h"
#include "subsystems/injector/Injector.h"
#include "subsystems/intake/Intake.h"
#include "subsystems/shooter/Shooter.h"

class RobotContainer {
 public:
  RobotContainer();

  std::optional<frc2::CommandPtr> GetAutonomousCommand();
  void Periodic();

  // turns off shooter, injector, agitator and intake rollers
  void StopMechanisms();

 private:
  void ConfigureBindings();
  frc2::CommandPtr Charge(Shooter::Location location);
  std::string ShotStatus() const;

  static std::unique_ptr<Controls> CreateControls();

  std::unique_ptr<Controls> m_controls = CreateControls();
  Drive m_drive;
  Shooter m_shooter;
  Injector m_injector;
  Agitator m_agitator;
  Intake m_intake;
  AutoChooser m_autoChooser{m_drive, m_shooter, m_injector, m_agitator, m_intake};
};
