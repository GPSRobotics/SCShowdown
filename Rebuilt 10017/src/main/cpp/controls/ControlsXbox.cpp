#include "controls/ControlsXbox.h"

ControlsXbox::ControlsXbox(int port) : m_controller{port} {}

double ControlsXbox::GetForward() { return -m_controller.GetLeftY(); }

double ControlsXbox::GetTurn() { return -m_controller.GetRightX(); }

// start is the three lines button (menu), back is the two squares button (view)
frc2::Trigger ControlsXbox::ToggleSlowMode() { return m_controller.Start(); }

frc2::Trigger ControlsXbox::ToggleTurnOnly() { return m_controller.Back(); }

frc2::Trigger ControlsXbox::ToggleArm() { return m_controller.B(); }

frc2::Trigger ControlsXbox::ToggleIntake() { return m_controller.LeftTrigger(); }

frc2::Trigger ControlsXbox::ToggleEject() { return m_controller.LeftBumper(); }

frc2::Trigger ControlsXbox::ToggleAgitator() { return m_controller.RightBumper(); }

frc2::Trigger ControlsXbox::ToggleUnjam() { return m_controller.Y(); }

frc2::Trigger ControlsXbox::StopAll() { return m_controller.X(); }

frc2::Trigger ControlsXbox::ToggleFullPowerShot() { return m_controller.RightTrigger(); }

frc2::Trigger ControlsXbox::ChargeHub() { return m_controller.POVUp(); }

frc2::Trigger ControlsXbox::ChargeTrench() { return m_controller.POVRight(); }

frc2::Trigger ControlsXbox::ChargeCorner() { return m_controller.POVLeft(); }

frc2::Trigger ControlsXbox::ChargeTower() { return m_controller.POVDown(); }
