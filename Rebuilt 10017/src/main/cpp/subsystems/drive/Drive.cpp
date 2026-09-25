#include "subsystems/drive/Drive.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPLTVController.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

#include "FieldConstants.h"

using namespace ctre::phoenix6;
using namespace DriveConstants;

namespace {
const char* ToString(Drive::SpeedMode mode) {
  switch (mode) {
    case Drive::SpeedMode::kNormal:
      return "Normal";
    case Drive::SpeedMode::kSlow:
      return "Slow";
    case Drive::SpeedMode::kTurnOnly:
      return "Turn Only";
  }
  return "Unknown";
}

void ConfigureMotor(hardware::TalonFX& motor, bool inverted) {
  configs::TalonFXConfiguration config;
  config.MotorOutput.Inverted = inverted ? signals::InvertedValue::Clockwise_Positive
                                         : signals::InvertedValue::CounterClockwise_Positive;
  motor.GetConfigurator().Apply(config);
}
}  // namespace

Drive::Drive() {
  ConfigureMotor(m_leftMotor, kLeftInverted);
  ConfigureMotor(m_rightMotor, kRightInverted);

  m_leftMotor.SetPosition(0_tr);
  m_rightMotor.SetPosition(0_tr);
  m_odometry.ResetPosition(GetHeading(), GetLeftDistance(), GetRightDistance(), kStartPose);

  ConfigureAutoBuilder();
  frc::SmartDashboard::PutData("Field", &m_field);

  if (frc::RobotBase::IsSimulation()) {
    m_sim = std::make_unique<DriveSim>(m_leftMotor, m_rightMotor);
  }
}

void Drive::Periodic() {
  BaseStatusSignal::RefreshAll(m_leftPosition, m_rightPosition, m_leftVelocity, m_rightVelocity);

  if (!m_gyroZeroed && !m_gyro.IsCalibrating()) {
    m_gyro.ZeroYaw();
    m_odometry.ResetPosition(GetHeading(), GetLeftDistance(), GetRightDistance(), GetPose());
    m_gyroZeroed = true;
  }

  m_odometry.Update(GetHeading(), GetLeftDistance(), GetRightDistance());

  frc::Pose2d pose = GetPose();
  m_field.SetRobotPose(pose);
  m_posePublisher.Set(pose);
  frc::SmartDashboard::PutNumber("Drive/LeftEncoder", m_leftPosition.GetValue().value());
  frc::SmartDashboard::PutNumber("Drive/RightEncoder", m_rightPosition.GetValue().value());
  // rounded so the dashboard doesnt show a bunch of decimals, /Drive/Pose has the exact pose
  frc::SmartDashboard::PutNumber("Odometry/X_m", std::round(pose.X().value() * 100.0) / 100.0);
  frc::SmartDashboard::PutNumber("Odometry/Y_m", std::round(pose.Y().value() * 100.0) / 100.0);
  frc::SmartDashboard::PutNumber("Odometry/Angle_deg",
                                 std::round(pose.Rotation().Degrees().value() * 10.0) / 10.0);
  frc::SmartDashboard::PutNumber("Odometry/DistToHub_m",
                                 std::round(GetDistanceToHub() * 100.0) / 100.0);
  frc::SmartDashboard::PutNumber("Gyro/Heading_deg", std::round(m_gyro.GetAngle() * 10.0) / 10.0);
  frc::SmartDashboard::PutString("Drive/Mode", ToString(m_speedMode));
}

void Drive::SimulationPeriodic() { m_sim->Update(); }

void Drive::ArcadeDrive(double forward, double turn) { SetPower(forward - turn, forward + turn); }

// called by PathPlanner every loop while following a path
void Drive::DriveWithSpeeds(const frc::ChassisSpeeds& speeds) {
  frc::DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.ToWheelSpeeds(speeds);
  wheelSpeeds.Desaturate(units::meters_per_second_t{kMaxSpeedMetersPerSecond});
  SetPower(wheelSpeeds.left.value() / kMaxSpeedMetersPerSecond,
           wheelSpeeds.right.value() / kMaxSpeedMetersPerSecond);
}

void Drive::Stop() { SetPower(0.0, 0.0); }

void Drive::SetPower(double left, double right) {
  m_leftMotor.SetControl(controls::DutyCycleOut{std::clamp(left, -1.0, 1.0)});
  m_rightMotor.SetControl(controls::DutyCycleOut{std::clamp(right, -1.0, 1.0)});
}

frc::Pose2d Drive::GetPose() const { return m_odometry.GetPose(); }

void Drive::ResetPose(const frc::Pose2d& pose) {
  m_odometry.ResetPosition(GetHeading(), GetLeftDistance(), GetRightDistance(), pose);
}

frc::ChassisSpeeds Drive::GetChassisSpeeds() const {
  return m_kinematics.ToChassisSpeeds(frc::DifferentialDriveWheelSpeeds{
      units::meters_per_second_t{m_leftVelocity.GetValue().value() * kMetersPerMotorRotation},
      units::meters_per_second_t{m_rightVelocity.GetValue().value() * kMetersPerMotorRotation}});
}

double Drive::GetDistanceToHub() const {
  frc::Translation2d hub = IsRedAlliance() ? FieldConstants::kRedHub : FieldConstants::kBlueHub;
  return GetPose().Translation().Distance(hub).value();
}

double Drive::GetDriveScale() const {
  switch (m_speedMode) {
    case SpeedMode::kSlow:
      return m_slowDriveScale.Get();
    case SpeedMode::kTurnOnly:
      return 0.0;
    case SpeedMode::kNormal:
      return 1.0;
  }
  return 1.0;
}

double Drive::GetTurnScale() const {
  if (m_speedMode == SpeedMode::kNormal) {
    return m_turnScale.Get();
  }
  return m_turnScale.Get() * m_slowTurnScale.Get();
}

frc2::CommandPtr Drive::ToggleSpeedModeCommand(SpeedMode mode) {
  return frc2::cmd::RunOnce(
      [this, mode] { m_speedMode = m_speedMode == mode ? SpeedMode::kNormal : mode; });
}

void Drive::ConfigureAutoBuilder() {
  pathplanner::AutoBuilder::configure(
      [this] { return GetPose(); }, [this](const frc::Pose2d& pose) { ResetPose(pose); },
      [this] { return GetChassisSpeeds(); },
      [this](const frc::ChassisSpeeds& speeds) { DriveWithSpeeds(speeds); },
      std::make_shared<pathplanner::PPLTVController>(20_ms),
      pathplanner::RobotConfig::fromGUISettings(), IsRedAlliance, this);

  pathplanner::PathPlannerLogging::setLogActivePathCallback(
      [this](const std::vector<frc::Pose2d>& poses) {
        m_field.GetObject("Path")->SetPoses(poses);
        m_pathPublisher.Set(poses);
      });
}

// navx is clockwise positive and odometry wants counterclockwise positive
frc::Rotation2d Drive::GetHeading() { return frc::Rotation2d{units::degree_t{-m_gyro.GetAngle()}}; }

units::meter_t Drive::GetLeftDistance() const {
  return units::meter_t{m_leftPosition.GetValue().value() * kMetersPerMotorRotation};
}

units::meter_t Drive::GetRightDistance() const {
  return units::meter_t{m_rightPosition.GetValue().value() * kMetersPerMotorRotation};
}

bool Drive::IsRedAlliance() {
  auto alliance = frc::DriverStation::GetAlliance();
  return alliance && *alliance == frc::DriverStation::Alliance::kRed;
}
