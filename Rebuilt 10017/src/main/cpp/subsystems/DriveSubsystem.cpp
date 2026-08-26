#include "subsystems/DriveSubsystem/DriveSubsystem.h"
#include "rev/config/SparkMaxConfig.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>

using namespace rev::spark;

DriveSubsystem::DriveSubsystem() {
    SparkMaxConfig centerConfig{};
    centerConfig.Inverted(false)
        .SetIdleMode(SparkMaxConfig::IdleMode::kBrake);
    m_centerMotor.Configure(centerConfig,
        SparkMax::ResetMode::kResetSafeParameters,
        SparkMax::PersistMode::kPersistParameters);

    frc::SmartDashboard::PutNumber("Left Drive", 0.0);
    frc::SmartDashboard::PutNumber("Right Drive", 0.0);
}

void DriveSubsystem::Drive(double leftSpeed, double rightSpeed, double strafeSpeed) {
    if (std::abs(strafeSpeed) < 0.1) strafeSpeed = 0.0;
    m_leftMotor.Set(leftSpeed);
    m_rightMotor.Set(rightSpeed);
    m_centerMotor.Set(strafeSpeed);
    frc::SmartDashboard::PutNumber("Left Motor Output", leftSpeed);
    frc::SmartDashboard::PutNumber("Right Motor Output", rightSpeed);
    frc::SmartDashboard::PutNumber("Strafe Output", strafeSpeed);
}

void DriveSubsystem::Periodic() {
    double left = frc::SmartDashboard::GetNumber("Left Drive", 0.0);
    double right = frc::SmartDashboard::GetNumber("Right Drive", 0.0);
    Drive(left, right, 0.0);
}