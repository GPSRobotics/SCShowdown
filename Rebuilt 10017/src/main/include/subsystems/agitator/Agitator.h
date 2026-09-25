#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>

#include "subsystems/agitator/AgitatorConstants.h"
#include "util/TunableNumber.h"

class Agitator : public frc2::SubsystemBase {
 public:
  enum class Goal { kIdle, kFeed, kReverse };
  enum class State { kIdle, kFeeding, kReversing, kDisabled };

  Agitator();

  void Periodic() override;

  void Feed();
  void Reverse();
  void Stop();
  void SetBrakeMode(bool brake);

  bool IsGoal(Goal goal) const { return m_goal == goal; }
  State GetState() const { return m_state; }
  double GetPower() const;
  double GetCurrent();

  frc2::CommandPtr ToggleFeedCommand();

 private:
  State EvaluateState() const;
  void ApplyOutputs();

  rev::spark::SparkMax m_motor;

  Goal m_goal = Goal::kIdle;
  State m_state = State::kIdle;

  TunableNumber m_feedPercent{"Agitator/Feed %", AgitatorConstants::kFeedPercent};
  TunableNumber m_reversePercent{"Agitator/Reverse %", AgitatorConstants::kReversePercent};
};
