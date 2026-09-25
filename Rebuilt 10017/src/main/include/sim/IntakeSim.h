#pragma once

#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/system/plant/DCMotor.h>
#include <rev/SparkMax.h>
#include <rev/sim/SparkMaxSim.h>
#include <units/angle.h>

#include "subsystems/intake/IntakeConstants.h"

// arm sim so the pivot pid, encoder and soft limits work in sim
class IntakeSim {
 public:
  explicit IntakeSim(rev::spark::SparkMax& pivotMotor);

  void Update();

 private:
  frc::DCMotor m_gearbox = frc::DCMotor::NEO(1);

  rev::spark::SparkMaxSim m_pivotSim;

  frc::sim::SingleJointedArmSim m_arm{
      m_gearbox,
      360.0 / IntakeConstants::Pivot::kDegreesPerMotorRotation,
      frc::sim::SingleJointedArmSim::EstimateMOI(IntakeConstants::Sim::kArmLength,
                                                 IntakeConstants::Sim::kArmMass),
      IntakeConstants::Sim::kArmLength,
      units::degree_t{IntakeConstants::Pivot::kMinAngleDeg},
      units::degree_t{IntakeConstants::Pivot::kMaxAngleDeg},
      false,  // no gravity, which way the arm hangs isn't known
      0_deg   // stowed at power-on
  };
};
