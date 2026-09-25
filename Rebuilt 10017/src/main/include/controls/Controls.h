#pragma once

#include <frc2/command/button/Trigger.h>

class Controls {
 public:
  virtual ~Controls() = default;

  virtual double GetForward() = 0;
  virtual double GetTurn() = 0;
  virtual frc2::Trigger ToggleSlowMode() = 0;
  virtual frc2::Trigger ToggleTurnOnly() = 0;

  virtual frc2::Trigger ToggleArm() = 0;
  virtual frc2::Trigger ToggleIntake() = 0;
  virtual frc2::Trigger ToggleEject() = 0;
  virtual frc2::Trigger ToggleAgitator() = 0;
  virtual frc2::Trigger ToggleUnjam() = 0;
  virtual frc2::Trigger StopAll() = 0;

  virtual frc2::Trigger ToggleFullPowerShot() = 0;
  virtual frc2::Trigger ChargeHub() = 0;
  virtual frc2::Trigger ChargeTrench() = 0;
  virtual frc2::Trigger ChargeCorner() = 0;
  virtual frc2::Trigger ChargeTower() = 0;
};
