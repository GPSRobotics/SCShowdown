#pragma once

#include <frc2/command/button/CommandXboxController.h>

#include "controls/Controls.h"

class ControlsXbox : public Controls {
 public:
  explicit ControlsXbox(int port);

  double GetForward() override;
  double GetTurn() override;
  frc2::Trigger ToggleSlowMode() override;
  frc2::Trigger ToggleTurnOnly() override;

  frc2::Trigger ToggleArm() override;
  frc2::Trigger ToggleIntake() override;
  frc2::Trigger ToggleEject() override;
  frc2::Trigger ToggleAgitator() override;
  frc2::Trigger ToggleUnjam() override;
  frc2::Trigger StopAll() override;

  frc2::Trigger ToggleFullPowerShot() override;
  frc2::Trigger ChargeHub() override;
  frc2::Trigger ChargeTrench() override;
  frc2::Trigger ChargeCorner() override;
  frc2::Trigger ChargeTower() override;

 protected:
  frc2::CommandXboxController m_controller;
};
