#pragma once
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/SparkMax.h>

class DriveSubsystem : public frc2::SubsystemBase {
public:
    DriveSubsystem();
    void Drive(double leftSpeed, double rightSpeed, double strafeSpeed);
    void Periodic() override;
private:
    ctre::phoenix6::hardware::TalonFX m_leftMotor{1};
    ctre::phoenix6::hardware::TalonFX m_rightMotor{2};
    rev::spark::SparkMax m_centerMotor{
        10, rev::spark::SparkLowLevel::MotorType::kBrushless};
};