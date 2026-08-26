#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/DriverStation.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/SparkMax.h>

#include <studica/AHRS.h>

// PathPlanner
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPLTVController.h>
#include <pathplanner/lib/config/RobotConfig.h>

#include "subsystems/DriveSubsystem/Constants.h"

class DriveSubsystem : public frc2::SubsystemBase {
public:
    DriveSubsystem();

    void Periodic() override;

    // ── Drive ──────────────────────────────────────────────────
    void ArcadeDrive(double speed, double rotation);
    void SetHWheelPower(double power);

    // Used by PathPlanner to drive the robot along a path
    void DriveWithSpeeds(frc::ChassisSpeeds speeds);

    // ── Odometry ───────────────────────────────────────────────
    frc::Pose2d GetPose() const;
    double      GetDistanceToHub() const;
    void        ResetOdometryToKnownPosition();
    void        ResetOdometry(const frc::Pose2d& pose);

    // Returns current wheel speeds for PathPlanner
    frc::ChassisSpeeds GetChassisSpeeds() const;

private:
    ctre::phoenix6::hardware::TalonFX leftMotor  { DriveConstants::kLeftMotorID  };
    ctre::phoenix6::hardware::TalonFX rightMotor { DriveConstants::kRightMotorID };
    rev::spark::SparkMax middleMotor { DriveConstants::kMiddleMotorID,
                                       rev::spark::SparkLowLevel::MotorType::kBrushless };

    ctre::phoenix6::StatusSignal<units::angle::turn_t>& leftPosition  = leftMotor.GetPosition();
    ctre::phoenix6::StatusSignal<units::angle::turn_t>& rightPosition = rightMotor.GetPosition();

    // Velocity signals needed for ChassisSpeeds
    ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t>& leftVelocity  = leftMotor.GetVelocity();
    ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t>& rightVelocity = rightMotor.GetVelocity();

    studica::AHRS m_gyro { studica::AHRS::NavXComType::kMXP_SPI };

    // Tracks whether the gyro has finished calibrating.
    // Odometry is not seeded until this is true.
    bool m_gyroCalibrated = false;

    frc::DifferentialDriveOdometry m_odometry {
        frc::Rotation2d{}, 0_m, 0_m, frc::Pose2d{}
    };

    // Kinematics needed to convert ChassisSpeeds ↔ wheel speeds
    frc::DifferentialDriveKinematics m_kinematics {
        DriveConstants::kTrackWidth
    };

    units::meter_t  GetLeftDistanceMeters()  const;
    units::meter_t  GetRightDistanceMeters() const;
    frc::Rotation2d GetHeading();
};