#include "sim/IntakeSim.h"

#include <frc/RobotController.h>
#include <units/angular_velocity.h>

IntakeSim::IntakeSim(rev::spark::SparkMax& pivotMotor) : m_pivotSim{&pivotMotor, &m_gearbox} {}

void IntakeSim::Update() {
  double battery = frc::RobotController::GetBatteryVoltage().value();
  m_arm.SetInputVoltage(units::volt_t{m_pivotSim.GetAppliedOutput() * battery});
  m_arm.Update(20_ms);

  // pivot encoder is in degrees
  m_pivotSim.iterate(units::degrees_per_second_t{m_arm.GetVelocity()}.value(), battery, 0.02);
}
