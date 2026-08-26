// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "GlobalConstants.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {

drive.SetDefaultCommand(
    frc2::cmd::Run([this] {
        double left = -m_driverController.GetLeftY();
        double right = -m_driverController.GetRightY();
        double strafe = m_driverController.GetLeftX();
        drive.Drive(left, right, strafe);
    }, {&drive})
);
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
  controller.X().WhileTrue(frc2::cmd::Run([this](){
      Intake.On();
  },{&Intake}));
  
  controller.Y().WhileTrue(frc2::cmd::Run([this](){
      Intake.Off();
  },{&Intake}));
  //d pad up On
  //d pad down Off

  m_driverController.RightTrigger().WhileTrue(frc2::cmd::Run([this](){
  shooter.ShooterShoot();
  }, {&shooter}));

  m_driverController.RightBumper().WhileTrue(frc2::cmd::Run([this](){
    shooter.ShooterStop();
  }, {&shooter}));

  m_driverController.LeftTrigger().WhileTrue(frc2::cmd::Run([this](){
    shooter.ShooterBack();
  }, {&shooter}));
  
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
}
 
