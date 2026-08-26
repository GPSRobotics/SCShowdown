#include "subsystems/ShooterSubsystem/ShooterSubsystem.h"
#include "rev/config/SparkFlexConfig.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace rev::spark;
using namespace frc;

ShooterSubsystem::ShooterSubsystem():
    kRightShooterMotor{ShooterConstants::kRightMotorPort, SparkLowLevel::MotorType::kBrushless},
    kLeftShooterMotor{ShooterConstants::kLeftMotorPort, SparkLowLevel::MotorType::kBrushless},
    m_rightController{kRightShooterMotor.GetClosedLoopController()},
    m_leftController{kLeftShooterMotor.GetClosedLoopController()},
    m_rightEncoder{kRightShooterMotor.GetEncoder()},
    m_leftEncoder{kLeftShooterMotor.GetEncoder()}
{       
    SparkFlexConfig rightConfig{};
    rightConfig.Inverted(false)
        .SetIdleMode(rev::spark::SparkFlexConfig::IdleMode::kCoast);
    rightConfig.closedLoop
        .P(ShooterConstants::KP)
        .I(ShooterConstants::KI)
        .D(ShooterConstants::KD)
        .VelocityFF(ShooterConstants::KFF);
    kRightShooterMotor.Configure(rightConfig, 
        SparkFlex::ResetMode::kResetSafeParameters,
        SparkFlex::PersistMode::kPersistParameters);

    SparkFlexConfig leftConfig{};
    leftConfig.Inverted(true)
        .SetIdleMode(rev::spark::SparkFlexConfig::IdleMode::kCoast);
    leftConfig.closedLoop
        .P(ShooterConstants::KP)
        .I(ShooterConstants::KI)
        .D(ShooterConstants::KD)
        .VelocityFF(ShooterConstants::KFF);
    kLeftShooterMotor.Configure(leftConfig,               
        SparkFlex::ResetMode::kResetSafeParameters,
        SparkFlex::PersistMode::kPersistParameters);            
}

    void ShooterSubsystem::Periodic() {
    }

    void ShooterSubsystem::ShooterShoot(){
        SetRPM(1000.0);
    }

    void ShooterSubsystem::ShooterStop(){
        SetRPM(0.0);
    }

    void ShooterSubsystem::ShooterBack(){
        SetRPM(-1000.0);
    }

    void ShooterSubsystem::SetRPM(double rpm){
        m_targetRPM = rpm;

        m_leftController.SetReference(rpm, rev::spark::SparkBase::ControlType::kVelocity);
        m_rightController.SetReference(rpm, rev::spark::SparkBase::ControlType::kVelocity);

    }

    double ShooterSubsystem::GetRPM(){
        return (m_leftEncoder.GetVelocity() + m_rightEncoder.GetVelocity()) / 2.0;
    
    }