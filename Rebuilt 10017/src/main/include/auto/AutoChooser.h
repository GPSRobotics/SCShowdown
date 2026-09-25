#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <networktables/IntegerTopic.h>

#include "subsystems/agitator/Agitator.h"
#include "subsystems/drive/Drive.h"
#include "subsystems/injector/Injector.h"
#include "subsystems/intake/Intake.h"
#include "subsystems/shooter/Shooter.h"

// auto is picked by number from /Auton/Selection or the Auton Chooser dropdown
//   0 = Shoot Hub (default)   3 = Shoot Corner
//   1 = Shoot Tower           4 = Right Side
//   2 = Shoot Trench          5 = Shoot Hub Right Side
//   9 = Do Nothing
//   10 and up = PathPlanner autos in abc order, ones with "Test" in the name need tuning mode on
class AutoChooser {
 public:
  AutoChooser(Drive& drive, Shooter& shooter, Injector& injector, Agitator& agitator,
              Intake& intake);

  std::optional<frc2::CommandPtr> GetSelected();
  void Periodic();

 private:
  std::optional<frc2::CommandPtr> BuildAuto(int number);
  std::string NameOf(int number) const;
  std::optional<std::string> PathPlannerName(int number) const;
  bool IsAvailable(int number) const;
  void BuildChooser();
  void UpdatePreview(int number, bool show);

  Drive& m_drive;
  Shooter& m_shooter;
  Injector& m_injector;
  Agitator& m_agitator;
  Intake& m_intake;

  nt::IntegerEntry m_selection;
  std::vector<std::string> m_pathPlannerAutos;

  // remade when tuning mode turns on or off since options cant be taken out of a chooser
  std::unique_ptr<frc::SendableChooser<int>> m_chooser;
  bool m_chooserHasTests = false;

  // -1 so the first update always draws
  int m_previewNumber = -1;
  bool m_previewRed = false;
  bool m_previewShown = false;
};
