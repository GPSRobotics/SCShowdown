#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandScheduler.h>
#include <optional>
#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
public:
    void RobotInit() override;
    void RobotPeriodic() override;

    void DisabledInit() override;
    void DisabledPeriodic() override;

    void TeleopInit() override;
    void TeleopPeriodic() override;

    void AutonomousInit() override;
    void AutonomousPeriodic() override;

private:
    std::unique_ptr<RobotContainer> m_container;
    std::optional<frc2::CommandPtr> m_autonomousCommand;
};