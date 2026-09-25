#include "sim/DriveSim.h"

#include <frc/RobotController.h>
#include <frc/simulation/SimDeviceSim.h>

using namespace ctre::phoenix6;
using namespace DriveConstants;

namespace {
units::turn_t ToMotorRotations(units::meter_t distance) {
  return units::turn_t{distance.value() / kMetersPerMotorRotation};
}

units::turns_per_second_t ToMotorRotations(units::meters_per_second_t speed) {
  return units::turns_per_second_t{speed.value() / kMetersPerMotorRotation};
}

sim::ChassisReference Orientation(bool inverted) {
  return inverted ? sim::ChassisReference::Clockwise_Positive
                  : sim::ChassisReference::CounterClockwise_Positive;
}
}  // namespace

DriveSim::DriveSim(hardware::TalonFX& leftMotor, hardware::TalonFX& rightMotor)
    : m_leftSim{leftMotor.GetSimState()}, m_rightSim{rightMotor.GetSimState()} {
  // motors are mounted the way their inverts say
  m_leftSim.Orientation = Orientation(kLeftInverted);
  m_rightSim.Orientation = Orientation(kRightInverted);
}

void DriveSim::Update() {
  units::volt_t battery = frc::RobotController::GetBatteryVoltage();
  m_leftSim.SetSupplyVoltage(battery);
  m_rightSim.SetSupplyVoltage(battery);

  m_drivetrain.SetInputs(m_leftSim.GetMotorVoltage(), m_rightSim.GetMotorVoltage());
  m_drivetrain.Update(20_ms);

  m_leftSim.SetRawRotorPosition(ToMotorRotations(m_drivetrain.GetLeftPosition()));
  m_leftSim.SetRotorVelocity(ToMotorRotations(m_drivetrain.GetLeftVelocity()));
  m_rightSim.SetRawRotorPosition(ToMotorRotations(m_drivetrain.GetRightPosition()));
  m_rightSim.SetRotorVelocity(ToMotorRotations(m_drivetrain.GetRightVelocity()));

  if (!m_gyroYaw) {
    FindGyro();
  }
  if (m_gyroYaw) {
    // navx is clockwise positive and the model is counterclockwise positive
    m_gyroYaw.Set(-m_drivetrain.GetHeading().Degrees().value());
  }
}

void DriveSim::FindGyro() {
  frc::sim::SimDeviceSim::EnumerateDevices(
      "navX-Sensor", [this](const char*, HAL_SimDeviceHandle handle) {
        m_gyroYaw = frc::sim::SimDeviceSim{handle}.GetDouble("Yaw");
      });
}
