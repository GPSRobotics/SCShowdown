#pragma once

#include <optional>
#include <string_view>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>

#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
 public:
  Robot();

  void RobotPeriodic() override;
  void AutonomousInit() override;
  void TeleopInit() override;

 private:
  void SwitchDashboardTab(std::string_view tab);

  RobotContainer m_container;
  std::optional<frc2::CommandPtr> m_autonomousCommand;

  // elastic switches to whatever tab name gets sent here
  nt::StringPublisher m_dashboardTab = nt::NetworkTableInstance::GetDefault()
                                           .GetStringTopic("/Elastic/SelectedTab")
                                           .Publish({.keepDuplicates = true});
};
