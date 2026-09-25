#include "controls/ControlsXboxSim.h"

#include <frc/DriverStation.h>

// this file is strictly because I (Carter from 10219) am on linux and my controller acts weird unless i do something like this file.
ControlsXboxSim::ControlsXboxSim(int port) : ControlsXbox{port} {}

double ControlsXboxSim::GetTurn() {
  if (UsingRawAxes()) {
    return -m_controller.GetHID().GetRawAxis(kRawRightXAxis);
  }
  return ControlsXbox::GetTurn();
}

frc2::Trigger ControlsXboxSim::ToggleFullPowerShot() {
  return frc2::Trigger{[this] {
    if (UsingRawAxes()) {
      return m_controller.GetHID().GetRawAxis(kRawRightTriggerAxis) > 0.0;
    }
    return m_controller.GetRightTriggerAxis() > 0.5;
  }};
}

bool ControlsXboxSim::UsingRawAxes() {
#ifdef __linux__
  return !frc::DriverStation::GetJoystickIsXbox(m_controller.GetHID().GetPort());
#else
  return false;
#endif
}
