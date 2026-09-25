#pragma once

#include <memory>

#include <frc/RobotBase.h>
#include <frc/filter/LinearFilter.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkFlex.h>
#include <rev/SparkRelativeEncoder.h>
#include <units/time.h>

#include "sim/ShooterSim.h"
#include "subsystems/shooter/ShooterConstants.h"
#include "util/TunableNumber.h"

class Shooter : public frc2::SubsystemBase {
 public:
  enum class Goal { kIdle, kVelocity, kFullPower };
  enum class State { kIdle, kSpinningUp, kAtSpeed, kFullPower, kDisabled };
  enum class Location { kNone, kHub, kTrench, kCorner, kTower };

  Shooter();

  void Periodic() override;
  void SimulationPeriodic() override;

  void SetRpm(double rpm);
  void SpinUpTo(Location location);
  void RunFullPower();
  void Stop();
  void SetBrakeMode(bool brake);

  bool AtSpeed() const;
  bool IsSpinningUpTo(Location location) const;
  double GetLocationRpm(Location location) const;
  double GetLocationInjectorPercent(Location location) const;
  units::second_t GetFullPowerFeedDelay() const;
  double GetAverageRpm() const;
  double GetLeftRpm() const;
  double GetRightRpm() const;
  double GetPower();
  State GetState() const { return m_state; }
  Location GetLocation() const { return m_location; }
  const char* GetLocationName() const;
  double GetTargetRpm() const { return m_targetRpm; }

  frc2::CommandPtr SpinUpCommand(double rpm);
  frc2::CommandPtr WaitUntilAtSpeedCommand(units::second_t timeout);
  frc2::CommandPtr StopCommand();

 private:
  State EvaluateState() const;
  void ApplyOutputs();
  void ConfigureClosedLoop();

  rev::spark::SparkFlex m_leftMotor;
  rev::spark::SparkFlex m_rightMotor;
  rev::spark::SparkClosedLoopController m_leftController;
  rev::spark::SparkClosedLoopController m_rightController;
  rev::spark::SparkRelativeEncoder m_leftEncoder;
  rev::spark::SparkRelativeEncoder m_rightEncoder;

  Goal m_goal = Goal::kIdle;
  State m_state = State::kIdle;
  double m_targetRpm = 0.0;
  Location m_location = Location::kNone;

  TunableNumber m_hubRpm{"Shooter/Hub RPM", ShooterConstants::kHubRpm};
  TunableNumber m_trenchRpm{"Shooter/Trench RPM", ShooterConstants::kTrenchRpm};
  TunableNumber m_cornerRpm{"Shooter/Corner RPM", ShooterConstants::kCornerRpm};
  TunableNumber m_towerRpm{"Shooter/Tower RPM", ShooterConstants::kTowerRpm};
  TunableNumber m_hubInjectorPercent{"Shooter/Hub Injector %",
                                     ShooterConstants::kHubInjectorPercent};
  TunableNumber m_trenchInjectorPercent{"Shooter/Trench Injector %",
                                        ShooterConstants::kTrenchInjectorPercent};
  TunableNumber m_cornerInjectorPercent{"Shooter/Corner Injector %",
                                        ShooterConstants::kCornerInjectorPercent};
  TunableNumber m_towerInjectorPercent{"Shooter/Tower Injector %",
                                       ShooterConstants::kTowerInjectorPercent};
  TunableNumber m_toleranceRpm{"Shooter/Tolerance RPM", ShooterConstants::kToleranceRpm};
  TunableNumber m_fullPowerFeedDelay{"Shooter/Full Power Feed Delay",
                                     ShooterConstants::kFullPowerFeedDelay.value()};
  TunableNumber m_kP{"Shooter/kP", ShooterConstants::kP};
  TunableNumber m_kI{"Shooter/kI", ShooterConstants::kI};
  TunableNumber m_kD{"Shooter/kD", ShooterConstants::kD};
  // sim needs its own kv, see ShooterConstants::Sim::kV
  TunableNumber m_kV{"Shooter/kV", frc::RobotBase::IsSimulation() ? ShooterConstants::Sim::kV
                                                                  : ShooterConstants::kV};

  double m_lastRpm = 0.0;
  double m_rpmPerSecond = 0.0;
  frc::LinearFilter<double> m_accelerationFilter = frc::LinearFilter<double>::MovingAverage(10);

  std::unique_ptr<ShooterSim> m_sim;
};
