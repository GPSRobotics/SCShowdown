#pragma once

#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <rev/SparkFlex.h>
#include <rev/sim/SparkFlexSim.h>

#include "subsystems/shooter/ShooterConstants.h"

// flywheel sim for each vortex so the pid and encoder work in sim
class ShooterSim {
 public:
  ShooterSim(rev::spark::SparkFlex& leftMotor, rev::spark::SparkFlex& rightMotor);

  void Update();

 private:
  frc::DCMotor m_gearbox = frc::DCMotor::NeoVortex(1);

  rev::spark::SparkFlexSim m_leftSim;
  rev::spark::SparkFlexSim m_rightSim;

  frc::sim::FlywheelSim m_leftFlywheel{
      frc::LinearSystemId::FlywheelSystem(m_gearbox, ShooterConstants::Sim::kFlywheelMoi, 1.0),
      m_gearbox};
  frc::sim::FlywheelSim m_rightFlywheel{
      frc::LinearSystemId::FlywheelSystem(m_gearbox, ShooterConstants::Sim::kFlywheelMoi, 1.0),
      m_gearbox};
};
