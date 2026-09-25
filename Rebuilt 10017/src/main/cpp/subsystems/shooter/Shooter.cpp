#include "subsystems/shooter/Shooter.h"

#include <cmath>

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <rev/config/SparkFlexConfig.h>

#include "subsystems/shooter/ShooterConstants.h"

using namespace rev::spark;

namespace {
const char* ToString(Shooter::State state) {
  switch (state) {
    case Shooter::State::kIdle:
      return "Idle";
    case Shooter::State::kSpinningUp:
      return "SpinningUp";
    case Shooter::State::kAtSpeed:
      return "AtSpeed";
    case Shooter::State::kFullPower:
      return "FullPower";
    case Shooter::State::kDisabled:
      return "Disabled";
  }
  return "Unknown";
}

const char* ToString(Shooter::Location location) {
  switch (location) {
    case Shooter::Location::kNone:
      return "None";
    case Shooter::Location::kHub:
      return "Hub";
    case Shooter::Location::kTrench:
      return "Trench";
    case Shooter::Location::kCorner:
      return "Corner";
    case Shooter::Location::kTower:
      return "Tower";
  }
  return "Unknown";
}

void ConfigureMotor(SparkFlex& motor, bool inverted) {
  SparkFlexConfig config;
  config.Inverted(inverted).SetIdleMode(SparkFlexConfig::IdleMode::kCoast);
  config.closedLoop.OutputRange(ShooterConstants::kMinOutput, ShooterConstants::kMaxOutput);
  motor.Configure(config, rev::ResetMode::kResetSafeParameters,
                  rev::PersistMode::kPersistParameters);
}
}  // namespace

Shooter::Shooter()
    : m_leftMotor{ShooterConstants::kLeftMotorId, SparkLowLevel::MotorType::kBrushless},
      m_rightMotor{ShooterConstants::kRightMotorId, SparkLowLevel::MotorType::kBrushless},
      m_leftController{m_leftMotor.GetClosedLoopController()},
      m_rightController{m_rightMotor.GetClosedLoopController()},
      m_leftEncoder{m_leftMotor.GetEncoder()},
      m_rightEncoder{m_rightMotor.GetEncoder()} {
  ConfigureMotor(m_leftMotor, ShooterConstants::kLeftInverted);
  ConfigureMotor(m_rightMotor, ShooterConstants::kRightInverted);
  ConfigureClosedLoop();

  if (frc::RobotBase::IsSimulation()) {
    m_sim = std::make_unique<ShooterSim>(m_leftMotor, m_rightMotor);
  }
}

void Shooter::Periodic() {
  // | instead of || so both get checked
  if (m_kP.HasChanged() | m_kI.HasChanged() | m_kD.HasChanged() | m_kV.HasChanged()) {
    ConfigureClosedLoop();
  }

  double rpm = GetAverageRpm();
  units::second_t loopTime = frc::TimedRobot::kDefaultPeriod;
  m_rpmPerSecond = m_accelerationFilter.Calculate((rpm - m_lastRpm) / loopTime.value());
  m_lastRpm = rpm;

  m_state = EvaluateState();
  ApplyOutputs();

  frc::SmartDashboard::PutString("Shooter/State", ToString(m_state));
  // rounded so the dashboard doesnt show a bunch of decimals
  frc::SmartDashboard::PutNumber("Shooter/LeftRPM", std::round(GetLeftRpm()));
  frc::SmartDashboard::PutNumber("Shooter/RightRPM", std::round(GetRightRpm()));
  frc::SmartDashboard::PutNumber("Shooter/TargetRPM", m_targetRpm);
  frc::SmartDashboard::PutBoolean("Shooter/AtSpeed", AtSpeed());
  frc::SmartDashboard::PutString("Shooter/Location", ToString(m_location));
}

void Shooter::SimulationPeriodic() { m_sim->Update(); }

Shooter::State Shooter::EvaluateState() const {
  if (frc::DriverStation::IsDisabled()) {
    return State::kDisabled;
  }

  switch (m_goal) {
    case Goal::kVelocity:
      return AtSpeed() ? State::kAtSpeed : State::kSpinningUp;
    case Goal::kFullPower:
      return State::kFullPower;
    case Goal::kIdle:
      return State::kIdle;
  }
  return State::kIdle;
}

void Shooter::ApplyOutputs() {
  switch (m_state) {
    case State::kSpinningUp:
    case State::kAtSpeed:
      m_leftController.SetSetpoint(m_targetRpm, SparkLowLevel::ControlType::kVelocity);
      m_rightController.SetSetpoint(m_targetRpm, SparkLowLevel::ControlType::kVelocity);
      break;
    case State::kFullPower:
      m_leftMotor.Set(ShooterConstants::kFullPower);
      m_rightMotor.Set(ShooterConstants::kFullPower);
      break;
    case State::kIdle:
    case State::kDisabled:
      m_leftMotor.StopMotor();
      m_rightMotor.StopMotor();
      break;
  }
}

void Shooter::SetRpm(double rpm) {
  m_targetRpm = rpm;
  m_location = Location::kNone;
  m_goal = Goal::kVelocity;
}

void Shooter::SpinUpTo(Location location) {
  m_targetRpm = GetLocationRpm(location);
  m_location = location;
  m_goal = Goal::kVelocity;
}

void Shooter::RunFullPower() {
  m_targetRpm = 0.0;
  m_location = Location::kNone;
  m_goal = Goal::kFullPower;
}

void Shooter::Stop() {
  m_targetRpm = 0.0;
  m_location = Location::kNone;
  m_goal = Goal::kIdle;
}

const char* Shooter::GetLocationName() const { return ToString(m_location); }

bool Shooter::IsSpinningUpTo(Location location) const {
  return m_goal == Goal::kVelocity && m_location == location;
}

double Shooter::GetLocationRpm(Location location) const {
  switch (location) {
    case Location::kHub:
      return m_hubRpm.Get();
    case Location::kTrench:
      return m_trenchRpm.Get();
    case Location::kCorner:
      return m_cornerRpm.Get();
    case Location::kTower:
      return m_towerRpm.Get();
    case Location::kNone:
      return 0.0;
  }
  return 0.0;
}

double Shooter::GetLocationInjectorPercent(Location location) const {
  switch (location) {
    case Location::kHub:
      return m_hubInjectorPercent.Get();
    case Location::kTrench:
      return m_trenchInjectorPercent.Get();
    case Location::kCorner:
      return m_cornerInjectorPercent.Get();
    case Location::kTower:
      return m_towerInjectorPercent.Get();
    case Location::kNone:
      return 0.0;
  }
  return 0.0;
}

units::second_t Shooter::GetFullPowerFeedDelay() const {
  return units::second_t{m_fullPowerFeedDelay.Get()};
}

// only changes the pid and kv so they can be tuned live
void Shooter::ConfigureClosedLoop() {
  SparkFlexConfig config;
  config.closedLoop.Pid(m_kP.Get(), m_kI.Get(), m_kD.Get());
  config.closedLoop.feedForward.kV(m_kV.Get());
  m_leftMotor.Configure(config, rev::ResetMode::kNoResetSafeParameters,
                        rev::PersistMode::kNoPersistParameters);
  m_rightMotor.Configure(config, rev::ResetMode::kNoResetSafeParameters,
                         rev::PersistMode::kNoPersistParameters);
}

// only changes the idle mode and everything else stays the same
void Shooter::SetBrakeMode(bool brake) {
  SparkFlexConfig config;
  config.SetIdleMode(brake ? SparkFlexConfig::IdleMode::kBrake : SparkFlexConfig::IdleMode::kCoast);
  m_leftMotor.Configure(config, rev::ResetMode::kNoResetSafeParameters,
                        rev::PersistMode::kNoPersistParameters);
  m_rightMotor.Configure(config, rev::ResetMode::kNoResetSafeParameters,
                         rev::PersistMode::kNoPersistParameters);
}

bool Shooter::AtSpeed() const {
  if (m_goal != Goal::kVelocity) {
    return false;
  }

  double tolerance = m_toleranceRpm.Get();
  bool atTarget = std::abs(GetLeftRpm() - m_targetRpm) < tolerance &&
                  std::abs(GetRightRpm() - m_targetRpm) < tolerance;

  double rpm = GetAverageRpm();
  bool maxedOut = rpm < m_targetRpm && rpm > m_targetRpm * ShooterConstants::kMaxedOutFraction &&
                  m_rpmPerSecond < ShooterConstants::kMaxedOutRpmPerSecond;

  return atTarget || maxedOut;
}

double Shooter::GetAverageRpm() const { return (GetLeftRpm() + GetRightRpm()) / 2.0; }

double Shooter::GetLeftRpm() const { return m_leftEncoder.GetVelocity(); }

double Shooter::GetRightRpm() const { return m_rightEncoder.GetVelocity(); }

double Shooter::GetPower() { return m_leftMotor.GetAppliedOutput(); }

frc2::CommandPtr Shooter::SpinUpCommand(double rpm) {
  return RunOnce([this, rpm] { SetRpm(rpm); });
}

frc2::CommandPtr Shooter::WaitUntilAtSpeedCommand(units::second_t timeout) {
  return frc2::cmd::WaitUntil([this] { return AtSpeed(); }).WithTimeout(timeout);
}

frc2::CommandPtr Shooter::StopCommand() {
  return RunOnce([this] { Stop(); });
}
