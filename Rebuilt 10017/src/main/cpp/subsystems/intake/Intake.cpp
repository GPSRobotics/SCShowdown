#include "subsystems/intake/Intake.h"

#include <cmath>

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkFlexConfig.h>
#include <rev/config/SparkMaxConfig.h>

#include "subsystems/intake/IntakeConstants.h"

using namespace rev::spark;
using namespace IntakeConstants;

namespace {
const char* ToString(Intake::State state) {
  switch (state) {
    case Intake::State::kIdle:
      return "Idle";
    case Intake::State::kStowing:
      return "Stowing";
    case Intake::State::kStowed:
      return "Stowed";
    case Intake::State::kDeploying:
      return "Deploying";
    case Intake::State::kDeployed:
      return "Deployed";
    case Intake::State::kDisabled:
      return "Disabled";
  }
  return "Unknown";
}

const char* ToString(Intake::RollerGoal goal) {
  switch (goal) {
    case Intake::RollerGoal::kOff:
      return "Off";
    case Intake::RollerGoal::kIntake:
      return "Intaking";
    case Intake::RollerGoal::kEject:
      return "Ejecting";
  }
  return "Unknown";
}
}  // namespace

Intake::Intake()
    : m_rollerMotor{Rollers::kMotorId, SparkLowLevel::MotorType::kBrushless},
      m_secondRollerMotor{Rollers::kSecondMotorId, SparkLowLevel::MotorType::kBrushless},
      m_pivotMotor{Pivot::kMotorId, SparkLowLevel::MotorType::kBrushless},
      m_pivotController{m_pivotMotor.GetClosedLoopController()},
      m_pivotEncoder{m_pivotMotor.GetEncoder()},
      m_targetAngleDeg{Pivot::kStowedAngleDeg} {
  SparkFlexConfig rollerConfig;
  rollerConfig.Inverted(Rollers::kInverted).SetIdleMode(SparkFlexConfig::IdleMode::kCoast);
  m_rollerMotor.Configure(rollerConfig, rev::ResetMode::kResetSafeParameters,
                          rev::PersistMode::kPersistParameters);

  // copies other roller because they spin the same
  SparkFlexConfig secondRollerConfig;
  secondRollerConfig.Follow(m_rollerMotor, Rollers::kSecondOpposesFirst)
      .SetIdleMode(SparkFlexConfig::IdleMode::kCoast);
  m_secondRollerMotor.Configure(secondRollerConfig, rev::ResetMode::kResetSafeParameters,
                                rev::PersistMode::kPersistParameters);

  SparkMaxConfig pivotConfig;
  pivotConfig.Inverted(Pivot::kInverted).SetIdleMode(SparkMaxConfig::IdleMode::kBrake);
  pivotConfig.encoder.PositionConversionFactor(Pivot::kDegreesPerMotorRotation)
      .VelocityConversionFactor(Pivot::kDegreesPerMotorRotation / 60.0);
  pivotConfig.softLimit.ForwardSoftLimit(Pivot::kMaxAngleDeg)
      .ForwardSoftLimitEnabled(true)
      .ReverseSoftLimit(Pivot::kMinAngleDeg)
      .ReverseSoftLimitEnabled(true);
  m_pivotMotor.Configure(pivotConfig, rev::ResetMode::kResetSafeParameters,
                         rev::PersistMode::kPersistParameters);
  ConfigurePivotClosedLoop();

  // arm has to be stowed when the robot turns on since that becomes 0
  m_pivotEncoder.SetPosition(0.0);

  if (frc::RobotBase::IsSimulation()) {
    m_sim = std::make_unique<IntakeSim>(m_pivotMotor);
  }
}

void Intake::Periodic() {
  // | instead of || so all of them get checked
  if (m_pivotKp.HasChanged() | m_pivotKi.HasChanged() | m_pivotKd.HasChanged() |
      m_pivotMaxOutput.HasChanged()) {
    ConfigurePivotClosedLoop();
  }

  m_state = EvaluateState();
  ApplyOutputs();

  frc::SmartDashboard::PutString("Intake/State", ToString(m_state));
  frc::SmartDashboard::PutString("Intake/Rollers", ToString(m_rollerGoal));
  frc::SmartDashboard::PutNumber("Intake/ArmAngle_deg", GetArmAngleDeg());
  frc::SmartDashboard::PutNumber("Intake/ArmTarget_deg", m_targetAngleDeg);
  frc::SmartDashboard::PutBoolean("Intake/ArmAtTarget", ArmNear(m_targetAngleDeg));
  frc::SmartDashboard::PutNumber("Intake/RollerPower", m_rollerMotor.Get());
}

void Intake::SimulationPeriodic() { m_sim->Update(); }

Intake::State Intake::EvaluateState() const {
  if (frc::DriverStation::IsDisabled()) {
    return State::kDisabled;
  }

  switch (m_armGoal) {
    case ArmGoal::kStow:
      return ArmNear(m_stowedAngle.Get()) ? State::kStowed : State::kStowing;
    case ArmGoal::kDeploy:
      return ArmNear(m_deployedAngle.Get()) ? State::kDeployed : State::kDeploying;
    case ArmGoal::kIdle:
      return State::kIdle;
  }
  return State::kIdle;
}

void Intake::ApplyOutputs() {
  switch (m_state) {
    case State::kStowing:
    case State::kStowed:
      SetArmAngle(m_stowedAngle.Get());
      break;
    case State::kDeploying:
    case State::kDeployed:
      SetArmAngle(m_deployedAngle.Get());
      break;
    case State::kIdle:
    case State::kDisabled:
      m_pivotMotor.StopMotor();
      break;
  }

  if (m_state == State::kDisabled) {
    m_rollerMotor.StopMotor();
    return;
  }

  switch (m_rollerGoal) {
    case RollerGoal::kOff:
      m_rollerMotor.StopMotor();
      break;
    case RollerGoal::kIntake:
      m_rollerMotor.Set(m_intakePercent.Get() / 100.0);
      break;
    case RollerGoal::kEject:
      m_rollerMotor.Set(-m_ejectPercent.Get() / 100.0);
      break;
  }
}

void Intake::StopRollers() { m_rollerGoal = RollerGoal::kOff; }

double Intake::GetArmAngleDeg() const { return m_pivotEncoder.GetPosition(); }

void Intake::SetArmAngle(double degrees) {
  m_targetAngleDeg = degrees;
  m_pivotController.SetSetpoint(degrees, SparkLowLevel::ControlType::kPosition);
}

bool Intake::ArmNear(double degrees) const {
  return std::abs(GetArmAngleDeg() - degrees) < Pivot::kToleranceDeg;
}

// only changes the pid and max output so they can be tuned live
void Intake::ConfigurePivotClosedLoop() {
  double maxOutput = m_pivotMaxOutput.Get();
  SparkMaxConfig config;
  config.closedLoop.Pid(m_pivotKp.Get(), m_pivotKi.Get(), m_pivotKd.Get())
      .OutputRange(-maxOutput, maxOutput);
  m_pivotMotor.Configure(config, rev::ResetMode::kNoResetSafeParameters,
                         rev::PersistMode::kNoPersistParameters);
}

frc2::CommandPtr Intake::IntakeCommand() {
  return RunOnce([this] {
    SetArmGoal(ArmGoal::kDeploy);
    SetRollerGoal(RollerGoal::kIntake);
  });
}

frc2::CommandPtr Intake::EjectCommand() {
  return RunOnce([this] {
    SetArmGoal(ArmGoal::kDeploy);
    SetRollerGoal(RollerGoal::kEject);
  });
}

frc2::CommandPtr Intake::StowCommand() {
  return RunOnce([this] {
    SetArmGoal(ArmGoal::kStow);
    SetRollerGoal(RollerGoal::kOff);
  });
}

frc2::CommandPtr Intake::ToggleArmCommand() {
  return RunOnce(
      [this] { SetArmGoal(IsArmGoal(ArmGoal::kDeploy) ? ArmGoal::kStow : ArmGoal::kDeploy); });
}

frc2::CommandPtr Intake::ToggleIntakeCommand() {
  return RunOnce([this] {
    SetRollerGoal(IsRollerGoal(RollerGoal::kIntake) ? RollerGoal::kOff : RollerGoal::kIntake);
  });
}

frc2::CommandPtr Intake::ToggleEjectCommand() {
  return RunOnce([this] {
    SetRollerGoal(IsRollerGoal(RollerGoal::kEject) ? RollerGoal::kOff : RollerGoal::kEject);
  });
}
