#pragma once

#include "controls/ControlsXbox.h"

// linux sim doesnt always recognize xbox controllers so right x and rt end up on other axes
class ControlsXboxSim : public ControlsXbox {
 public:
  explicit ControlsXboxSim(int port);

  double GetTurn() override;
  frc2::Trigger ToggleFullPowerShot() override;

 private:
  static constexpr int kRawRightXAxis = 3;
  static constexpr int kRawRightTriggerAxis = 5;

  bool UsingRawAxes();
};
