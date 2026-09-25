#include "subsystems/agitator/Agitator.h"

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkMaxConfig.h>

#include "subsystems/agitator/AgitatorConstants.h"

using namespace rev::spark;

namespace {
const char* ToString(Agitator::State state) {
  switch (state) {
    case Agitator::State::kIdle:
      return "Idle";
    case Agitator::State::kFeeding:
      return "Feeding";
    case Agitator::State::kReversing:
      return "Reversing";
    case Agitator::State::kDisabled:
      return "Disabled";
  }
  return "Unknown";
}
}  // namespace

Agitator::Agitator() : m_motor{AgitatorConstants::kMotorId, SparkLowLevel::MotorType::kBrushless} {
  SparkMaxConfig config;
  config.Inverted(AgitatorConstants::kInverted).SetIdleMode(SparkMaxConfig::IdleMode::kCoast);
  m_motor.Configure(config, rev::ResetMode::kResetSafeParameters,
                    rev::PersistMode::kPersistParameters);
}

void Agitator::Periodic() {
  m_state = EvaluateState();
  ApplyOutputs();

  frc::SmartDashboard::PutString("Agitator/State", ToString(m_state));
  frc::SmartDashboard::PutNumber("Agitator/Power", GetPower());
  frc::SmartDashboard::PutNumber("Agitator Current", GetCurrent());
}

Agitator::State Agitator::EvaluateState() const {
  if (frc::DriverStation::IsDisabled()) {
    return State::kDisabled;
  }

  switch (m_goal) {
    case Goal::kFeed:
      return State::kFeeding;
    case Goal::kReverse:
      return State::kReversing;
    case Goal::kIdle:
      return State::kIdle;
  }
  return State::kIdle;
}

void Agitator::ApplyOutputs() {
  switch (m_state) {
    case State::kFeeding:
      m_motor.Set(m_feedPercent.Get() / 100.0);
      break;
    case State::kReversing:
      m_motor.Set(-m_reversePercent.Get() / 100.0);
      break;
    case State::kIdle:
    case State::kDisabled:
      m_motor.StopMotor();
      break;
  }
}

void Agitator::Feed() { m_goal = Goal::kFeed; }

void Agitator::Reverse() { m_goal = Goal::kReverse; }

void Agitator::Stop() { m_goal = Goal::kIdle; }

void Agitator::SetBrakeMode(bool brake) {
  SparkMaxConfig config;
  config.SetIdleMode(brake ? SparkMaxConfig::IdleMode::kBrake : SparkMaxConfig::IdleMode::kCoast);
  m_motor.Configure(config, rev::ResetMode::kNoResetSafeParameters,
                    rev::PersistMode::kNoPersistParameters);
}

double Agitator::GetPower() const { return m_motor.Get(); }

double Agitator::GetCurrent() { return m_motor.GetOutputCurrent(); }

frc2::CommandPtr Agitator::ToggleFeedCommand() {
  return RunOnce([this] {
    if (IsGoal(Goal::kFeed)) {
      Stop();
    } else {
      Feed();
    }
  });
}
