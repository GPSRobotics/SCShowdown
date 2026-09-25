#pragma once

#include <memory>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>

#include "sim/IntakeSim.h"
#include "subsystems/intake/IntakeConstants.h"
#include "util/TunableNumber.h"

class Intake : public frc2::SubsystemBase {
 public:
  enum class ArmGoal { kIdle, kStow, kDeploy };
  enum class RollerGoal { kOff, kIntake, kEject };
  enum class State { kIdle, kStowing, kStowed, kDeploying, kDeployed, kDisabled };

  Intake();

  void Periodic() override;
  void SimulationPeriodic() override;

  void SetArmGoal(ArmGoal goal) { m_armGoal = goal; }
  void SetRollerGoal(RollerGoal goal) { m_rollerGoal = goal; }
  bool IsArmGoal(ArmGoal goal) const { return m_armGoal == goal; }
  bool IsRollerGoal(RollerGoal goal) const { return m_rollerGoal == goal; }
  State GetState() const { return m_state; }

  // stops rollers but keeps arm at position
  void StopRollers();

  double GetArmAngleDeg() const;

  // autos move the arm and set the rollers together
  frc2::CommandPtr IntakeCommand();
  frc2::CommandPtr EjectCommand();
  frc2::CommandPtr StowCommand();

  // driver controls the arm and rollers separately
  frc2::CommandPtr ToggleArmCommand();
  frc2::CommandPtr ToggleIntakeCommand();
  frc2::CommandPtr ToggleEjectCommand();

 private:
  State EvaluateState() const;
  void ApplyOutputs();
  void SetArmAngle(double degrees);
  bool ArmNear(double degrees) const;
  void ConfigurePivotClosedLoop();

  rev::spark::SparkFlex m_rollerMotor;
  rev::spark::SparkFlex m_secondRollerMotor;
  rev::spark::SparkMax m_pivotMotor;
  rev::spark::SparkClosedLoopController m_pivotController;
  rev::spark::SparkRelativeEncoder m_pivotEncoder;

  ArmGoal m_armGoal = ArmGoal::kIdle;
  RollerGoal m_rollerGoal = RollerGoal::kOff;
  State m_state = State::kIdle;
  double m_targetAngleDeg;

  TunableNumber m_intakePercent{"Intake/Intake %", IntakeConstants::Rollers::kIntakePercent};
  TunableNumber m_ejectPercent{"Intake/Eject %", IntakeConstants::Rollers::kEjectPercent};
  TunableNumber m_stowedAngle{"Intake/Stowed Angle", IntakeConstants::Pivot::kStowedAngleDeg};
  TunableNumber m_deployedAngle{"Intake/Deployed Angle", IntakeConstants::Pivot::kDeployedAngleDeg};
  TunableNumber m_pivotKp{"Intake/Pivot kP", IntakeConstants::Pivot::kP};
  TunableNumber m_pivotKi{"Intake/Pivot kI", IntakeConstants::Pivot::kI};
  TunableNumber m_pivotKd{"Intake/Pivot kD", IntakeConstants::Pivot::kD};
  TunableNumber m_pivotMaxOutput{"Intake/Pivot Max Output", IntakeConstants::Pivot::kMaxOutput};

  std::unique_ptr<IntakeSim> m_sim;
};
