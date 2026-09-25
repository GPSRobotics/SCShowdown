#pragma once

#include <memory>

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
#include <studica/AHRS.h>
#include <units/length.h>

#include "sim/DriveSim.h"
#include "subsystems/drive/DriveConstants.h"
#include "util/TunableNumber.h"

class Drive : public frc2::SubsystemBase {
 public:
  enum class SpeedMode { kNormal, kSlow, kTurnOnly };

  Drive();

  void Periodic() override;
  void SimulationPeriodic() override;

  // positive turn turns left
  void ArcadeDrive(double forward, double turn);
  void DriveWithSpeeds(const frc::ChassisSpeeds& speeds);
  void Stop();

  frc::Pose2d GetPose() const;
  void ResetPose(const frc::Pose2d& pose);
  frc::ChassisSpeeds GetChassisSpeeds() const;
  double GetDistanceToHub() const;
  frc::Field2d& GetField() { return m_field; }

  double GetDriveScale() const;
  double GetTurnScale() const;
  // turns the mode on, or back to normal if its already on
  frc2::CommandPtr ToggleSpeedModeCommand(SpeedMode mode);

 private:
  void ConfigureAutoBuilder();
  void SetPower(double left, double right);
  frc::Rotation2d GetHeading();
  units::meter_t GetLeftDistance() const;
  units::meter_t GetRightDistance() const;

  static bool IsRedAlliance();

  ctre::phoenix6::hardware::TalonFX m_leftMotor{DriveConstants::kLeftMotorId};
  ctre::phoenix6::hardware::TalonFX m_rightMotor{DriveConstants::kRightMotorId};
  ctre::phoenix6::StatusSignal<units::turn_t>& m_leftPosition = m_leftMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turn_t>& m_rightPosition = m_rightMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t>& m_leftVelocity =
      m_leftMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t>& m_rightVelocity =
      m_rightMotor.GetVelocity();

  studica::AHRS m_gyro{studica::AHRS::NavXComType::kMXP_SPI};
  bool m_gyroZeroed = false;

  frc::DifferentialDriveKinematics m_kinematics{DriveConstants::kTrackWidth};
  frc::DifferentialDriveOdometry m_odometry{frc::Rotation2d{}, 0_m, 0_m};
  frc::Field2d m_field;

  // stuff for advantagescope
  nt::StructPublisher<frc::Pose2d> m_posePublisher =
      nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("/Drive/Pose").Publish();
  nt::StructArrayPublisher<frc::Pose2d> m_pathPublisher =
      nt::NetworkTableInstance::GetDefault()
          .GetStructArrayTopic<frc::Pose2d>("/Drive/ActivePath")
          .Publish();

  SpeedMode m_speedMode = SpeedMode::kNormal;

  TunableNumber m_turnScale{"Drive/Turn Scale", DriveConstants::JoystickDrive::kTurnScale};
  TunableNumber m_slowDriveScale{"Drive/Slow Drive Scale",
                                 DriveConstants::JoystickDrive::kSlowDriveScale};
  TunableNumber m_slowTurnScale{"Drive/Slow Turn Scale",
                                DriveConstants::JoystickDrive::kSlowTurnScale};

  std::unique_ptr<DriveSim> m_sim;
};
