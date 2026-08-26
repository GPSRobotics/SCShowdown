#include "Robot.h"

void Robot::RobotInit() {
    m_container = std::make_unique<RobotContainer>();
}

void Robot::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();

    m_container->UpdateDashboard();
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TeleopInit() {
    if (m_autonomousCommand) {
        m_autonomousCommand->Cancel();
    }
}

void Robot::TeleopPeriodic() {}

void Robot::AutonomousInit() {
    m_autonomousCommand = m_container->GetAutonomousCommand();
    if (m_autonomousCommand) {
        m_autonomousCommand->Schedule();
    }
}

void Robot::AutonomousPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif