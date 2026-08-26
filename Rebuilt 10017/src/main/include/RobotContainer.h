// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/CommandXboxController.h>

#include "GlobalConstants.h"
#include "subsystems/IntakeSubsystem/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem/ShooterSubsystem.h"

#include "subsystems/DriveSubsystem/DriveSubsystem.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:

  ShooterSubsystem shooter{};
  DriveSubsystem drive{};

  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_driverController{
      OperatorConstants::kDriverControllerPort};

  // The robot's subsystems are defined here...
  IntakeSubsystem Intake;

  void ConfigureBindings();
};
