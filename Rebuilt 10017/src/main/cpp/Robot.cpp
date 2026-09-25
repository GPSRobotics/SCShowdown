#include "Robot.h"

#include <string>

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>

Robot::Robot() {
  // gets rid of stupid fucking annoying connection warnings
  frc::DriverStation::SilenceJoystickConnectionWarning(true);

  // logging stuff, plug in usb drive to benefit
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());

  // toggle in elastic that turns tab switching on and off
  frc::SmartDashboard::SetDefaultBoolean("Auto Switch Tabs", true);

  // prints commands start and stop
  auto& scheduler = frc2::CommandScheduler::GetInstance();
  scheduler.OnCommandInitialize([](const frc2::Command& command) {
    frc::DataLogManager::Log("Started " + command.GetName());
  });
  scheduler.OnCommandFinish([](const frc2::Command& command) {
    frc::DataLogManager::Log("Finished " + command.GetName());
  });
  scheduler.OnCommandInterrupt([](const frc2::Command& command) {
    frc::DataLogManager::Log("Interrupted " + command.GetName());
  });
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  m_container.Periodic();

  std::string autoStatus = "Waiting for auto";
  if (m_autonomousCommand) {
    autoStatus = m_autonomousCommand->IsScheduled() ? "Running" : "Finished";
  }
  frc::SmartDashboard::PutString("Auton/Status", autoStatus);
}

void Robot::AutonomousInit() {
  SwitchDashboardTab("Autonomous");
  m_autonomousCommand = m_container.GetAutonomousCommand();
  if (m_autonomousCommand) {
    frc2::CommandScheduler::GetInstance().Schedule(*m_autonomousCommand);
  } else {
    frc::DataLogManager::Log("No auto selected");
  }
}

void Robot::TeleopInit() {
  SwitchDashboardTab("Teleop");
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
  m_container.StopMechanisms();
}

void Robot::SwitchDashboardTab(std::string_view tab) {
  if (frc::SmartDashboard::GetBoolean("Auto Switch Tabs", true)) {
    m_dashboardTab.Set(tab);
  }
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
