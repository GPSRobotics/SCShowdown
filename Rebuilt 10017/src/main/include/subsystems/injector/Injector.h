#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>

#include "subsystems/injector/InjectorConstants.h"
#include "util/TunableNumber.h"

class Injector : public frc2::SubsystemBase {
 public:
  enum class Goal { kIdle, kFeed, kCharge, kReverse };
  enum class State { kIdle, kFeeding, kCharging, kReversing, kDisabled };

  Injector();

  void Periodic() override;

  // full power toward the shooter
  void Feed();
  // toward the shooter at this percent
  void Charge(double percent);
  void Reverse();
  void Stop();
  void SetBrakeMode(bool brake);

  bool IsGoal(Goal goal) const { return m_goal == goal; }
  State GetState() const { return m_state; }
  double GetPower() const;

 private:
  State EvaluateState() const;
  void ApplyOutputs();

  rev::spark::SparkMax m_motor;

  Goal m_goal = Goal::kIdle;
  State m_state = State::kIdle;
  double m_chargePercent = 0.0;

  TunableNumber m_reversePercent{"Injector/Reverse %", InjectorConstants::kReversePercent};
};
