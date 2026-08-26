#pragma once

#include "subsystems/ShooterSubsystem/Constants.h"

#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/SparkFlex.h>

class ShooterSubsystem : public frc2::SubsystemBase {
public:
    ShooterSubsystem();

    void Periodic() override;

    void ShooterShoot();
    void ShooterStop();
    void ShooterBack();

    void SetRPM(double rpm);
    double GetRPM();

private:
    rev::spark::SparkFlex kRightShooterMotor;  // right first
    rev::spark::SparkFlex kLeftShooterMotor;   // left second
    rev::spark::SparkClosedLoopController m_rightController;
    rev::spark::SparkClosedLoopController m_leftController;
    rev::spark::SparkRelativeEncoder m_rightEncoder;
    rev::spark::SparkRelativeEncoder m_leftEncoder;
    double m_targetRPM = 100.0;
};
