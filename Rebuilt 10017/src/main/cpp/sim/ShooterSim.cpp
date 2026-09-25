#include "sim/ShooterSim.h"

#include <frc/RobotController.h>
#include <units/angular_velocity.h>

using namespace rev::spark;

namespace {
void UpdateSide(SparkFlexSim& motor, frc::sim::FlywheelSim& flywheel) {
  double battery = frc::RobotController::GetBatteryVoltage().value();
  flywheel.SetInputVoltage(units::volt_t{motor.GetAppliedOutput() * battery});
  flywheel.Update(20_ms);
  motor.iterate(units::revolutions_per_minute_t{flywheel.GetAngularVelocity()}.value(), battery,
                0.02);
}
}  // namespace

ShooterSim::ShooterSim(SparkFlex& leftMotor, SparkFlex& rightMotor)
    : m_leftSim{&leftMotor, &m_gearbox}, m_rightSim{&rightMotor, &m_gearbox} {}

void ShooterSim::Update() {
  UpdateSide(m_leftSim, m_leftFlywheel);
  UpdateSide(m_rightSim, m_rightFlywheel);
}
