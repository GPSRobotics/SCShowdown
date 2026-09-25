#include "subsystems/injector/Injector.h"

#include <algorithm>

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkMaxConfig.h>

#include "subsystems/injector/InjectorConstants.h"

using namespace rev::spark;

namespace {
const char* ToString(Injector::State state) {
  switch (state) {
    case Injector::State::kIdle:
      return "Idle";
    case Injector::State::kFeeding:
      return "Feeding";
    case Injector::State::kCharging:
      return "Charging";
    case Injector::State::kReversing:
      return "Reversing";
    case Injector::State::kDisabled:
      return "Disabled";
  }
  return "Unknown";
}
}  // namespace

Injector::Injector() : m_motor{InjectorConstants::kMotorId, SparkLowLevel::MotorType::kBrushless} {
  // brake so balls dont slowly get pushed up although they still might be able to idk
  SparkMaxConfig config;
  config.Inverted(InjectorConstants::kInverted).SetIdleMode(SparkMaxConfig::IdleMode::kBrake);
  m_motor.Configure(config, rev::ResetMode::kResetSafeParameters,
                    rev::PersistMode::kPersistParameters);
}

void Injector::Periodic() {
  m_state = EvaluateState();
  ApplyOutputs();

  frc::SmartDashboard::PutString("Injector/State", ToString(m_state));
  frc::SmartDashboard::PutNumber("Injector/Power", GetPower());
}

Injector::State Injector::EvaluateState() const {
  if (frc::DriverStation::IsDisabled()) {
    return State::kDisabled;
  }

  switch (m_goal) {
    case Goal::kFeed:
      return State::kFeeding;
    case Goal::kCharge:
      return State::kCharging;
    case Goal::kReverse:
      return State::kReversing;
    case Goal::kIdle:
      return State::kIdle;
  }
  return State::kIdle;
}

void Injector::ApplyOutputs() {
  switch (m_state) {
    case State::kFeeding:
      m_motor.Set(InjectorConstants::kShootDirection);
      break;
    case State::kCharging:
      m_motor.Set(InjectorConstants::kShootDirection * m_chargePercent / 100.0);
      break;
    case State::kReversing:
      m_motor.Set(-InjectorConstants::kShootDirection * m_reversePercent.Get() / 100.0);
      break;
    case State::kIdle:
    case State::kDisabled:
      m_motor.StopMotor();
      break;
  }
}

void Injector::Feed() { m_goal = Goal::kFeed; }

void Injector::Reverse() { m_goal = Goal::kReverse; }

void Injector::Charge(double percent) {
  m_chargePercent = std::clamp(percent, 0.0, 100.0);
  m_goal = Goal::kCharge;
}

void Injector::Stop() { m_goal = Goal::kIdle; }

// only changes the idle mode
void Injector::SetBrakeMode(bool brake) {
  SparkMaxConfig config;
  config.SetIdleMode(brake ? SparkMaxConfig::IdleMode::kBrake : SparkMaxConfig::IdleMode::kCoast);
  m_motor.Configure(config, rev::ResetMode::kNoResetSafeParameters,
                    rev::PersistMode::kNoPersistParameters);
}

double Injector::GetPower() const { return m_motor.Get(); }
