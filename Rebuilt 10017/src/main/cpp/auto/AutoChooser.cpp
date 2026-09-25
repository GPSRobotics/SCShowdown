#include "auto/AutoChooser.h"

#include <algorithm>
#include <array>
#include <string>
#include <utility>
#include <vector>

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/path/PathPlannerPath.h>

#include "auto/AutoCommands.h"
#include "util/TuningMode.h"

namespace {
// autos written in code, PathPlanner autos get numbers starting at kFirstPathPlannerNumber
struct AutoOption {
  int number;
  const char* name;
  // where to put the robot and what it does, shown on the dashboard
  const char* start;
  const char* plan;
  // drawn on the field before the match, empty if it doesnt drive
  const char* path;
};

constexpr std::array<AutoOption, 7> kAutos{{
    {0, "Shoot Hub", "At the hub shooting spot", "Hub shot for 10 s, intake down and ejecting", ""},
    {1, "Shoot Tower", "At the tower shooting spot",
     "Tower shot for 10 s, intake down and ejecting", ""},
    {2, "Shoot Trench", "At the trench shooting spot",
     "Trench shot for 10 s, intake down and ejecting", ""},
    {3, "Shoot Corner", "At the corner shooting spot",
     "Corner shot for 10 s, intake down and ejecting", ""},
    {4, "Right Side", "Start of the Right Side path (see field)",
     "Hub shot 5 s, drives the path intaking, corner shot 4 s", "Right Side"},
    {5, "Shoot Hub Right Side", "Right side of the hub",
     "4700 RPM shot for 10 s, intake down and ejecting", ""},
    {9, "Do Nothing", "Anywhere", "Sits still", ""},
}};

constexpr int kFirstPathPlannerNumber = 10;

const AutoOption* FindCodeAuto(int number) {
  for (const auto& autoOption : kAutos) {
    if (autoOption.number == number) {
      return &autoOption;
    }
  }
  return nullptr;
}

bool IsTestAuto(const std::string& name) { return name.find("Test") != std::string::npos; }
}  // namespace

AutoChooser::AutoChooser(Drive& drive, Shooter& shooter, Injector& injector, Agitator& agitator,
                         Intake& intake)
    : m_drive{drive},
      m_shooter{shooter},
      m_injector{injector},
      m_agitator{agitator},
      m_intake{intake},
      m_selection{
          nt::NetworkTableInstance::GetDefault().GetIntegerTopic("/Auton/Selection").GetEntry(0)} {
  // has to happen before any PathPlanner auto gets built
  AutoCommands::RegisterNamedCommands(shooter, injector, agitator, intake);

  m_pathPlannerAutos = pathplanner::AutoBuilder::getAllAutoNames();
  std::sort(m_pathPlannerAutos.begin(), m_pathPlannerAutos.end());

  // lists every auto number so they can be looked up when typing one in
  for (const auto& autoOption : kAutos) {
    frc::SmartDashboard::PutString("Auton/" + std::to_string(autoOption.number), autoOption.name);
  }
  for (size_t i = 0; i < m_pathPlannerAutos.size(); i++) {
    frc::SmartDashboard::PutString("Auton/" + std::to_string(kFirstPathPlannerNumber + i),
                                   m_pathPlannerAutos[i]);
  }

  m_selection.Set(0);
  BuildChooser();
}

std::optional<frc2::CommandPtr> AutoChooser::GetSelected() {
  int number = static_cast<int>(m_selection.Get());
  if (!IsAvailable(number)) {
    frc::DataLogManager::Log(NameOf(number) +
                             " isnt available, turn on tuning mode for test autos");
    return std::nullopt;
  }
  std::optional<frc2::CommandPtr> autoCommand = BuildAuto(number);
  if (!autoCommand) {
    return std::nullopt;
  }
  return std::move(*autoCommand)
      .BeforeStarting([this] { m_drive.Stop(); })
      .FinallyDo([this] { m_drive.Stop(); })
      .WithName("Auto: " + NameOf(number));
}

std::optional<frc2::CommandPtr> AutoChooser::BuildAuto(int number) {
  switch (number) {
    case 0:
      return AutoCommands::ShootHub(m_shooter, m_injector, m_agitator, m_intake);
    case 1:
      return AutoCommands::ShootTower(m_shooter, m_injector, m_agitator, m_intake);
    case 2:
      return AutoCommands::ShootTrench(m_shooter, m_injector, m_agitator, m_intake);
    case 3:
      return AutoCommands::ShootCorner(m_shooter, m_injector, m_agitator, m_intake);
    case 4:
      return AutoCommands::RightSide(m_shooter, m_injector, m_agitator, m_intake);
    case 5:
      return AutoCommands::ShootHubRight(m_shooter, m_injector, m_agitator, m_intake);
  }

  if (auto name = PathPlannerName(number)) {
    return pathplanner::PathPlannerAuto(*name).ToPtr();
  }
  return std::nullopt;
}

std::string AutoChooser::NameOf(int number) const {
  if (const AutoOption* autoOption = FindCodeAuto(number)) {
    return autoOption->name;
  }
  return PathPlannerName(number).value_or("Unknown");
}

std::optional<std::string> AutoChooser::PathPlannerName(int number) const {
  int index = number - kFirstPathPlannerNumber;
  if (index < 0 || index >= static_cast<int>(m_pathPlannerAutos.size())) {
    return std::nullopt;
  }
  return m_pathPlannerAutos[index];
}

bool AutoChooser::IsAvailable(int number) const {
  if (FindCodeAuto(number)) {
    return true;
  }
  auto name = PathPlannerName(number);
  return name && (!IsTestAuto(*name) || TuningMode::IsOn());
}

void AutoChooser::BuildChooser() {
  m_chooserHasTests = TuningMode::IsOn();

  int selected = static_cast<int>(m_selection.Get());
  int defaultNumber = IsAvailable(selected) ? selected : 0;

  auto chooser = std::make_unique<frc::SendableChooser<int>>();
  auto addOption = [&](int number) {
    if (number == defaultNumber) {
      chooser->SetDefaultOption(NameOf(number), number);
    } else if (IsAvailable(number)) {
      chooser->AddOption(NameOf(number), number);
    }
  };
  for (const auto& autoOption : kAutos) {
    addOption(autoOption.number);
  }
  for (size_t i = 0; i < m_pathPlannerAutos.size(); i++) {
    addOption(kFirstPathPlannerNumber + static_cast<int>(i));
  }
  chooser->OnChange([this](int number) { m_selection.Set(number); });

  frc::SmartDashboard::PutData("Auton Chooser", chooser.get());
  m_chooser = std::move(chooser);
  m_selection.Set(defaultNumber);
}

void AutoChooser::Periodic() {
  if (TuningMode::IsOn() != m_chooserHasTests) {
    BuildChooser();
  }

  int selected = static_cast<int>(m_selection.Get());
  const AutoOption* codeAuto = FindCodeAuto(selected);
  std::string start = "Unknown auto number";
  std::string plan = "";
  if (codeAuto) {
    start = codeAuto->start;
    plan = codeAuto->plan;
  } else if (PathPlannerName(selected) && !IsAvailable(selected)) {
    start = "Test auto, turn on tuning mode to use it";
  } else if (PathPlannerName(selected)) {
    start = "Start of its first path (see field)";
    plan = "Made in PathPlanner";
  }

  frc::SmartDashboard::PutNumber("Auton/SelectionNumber", selected);
  frc::SmartDashboard::PutString("Auton/Selected", NameOf(selected));
  frc::SmartDashboard::PutString("Auton/Start", start);
  frc::SmartDashboard::PutString("Auton/Plan", plan);

  // hide the preview once teleop starts
  UpdatePreview(selected, !frc::DriverStation::IsTeleopEnabled());
}

// draws the autos paths on the field so the drivers can see where to put the robot
void AutoChooser::UpdatePreview(int number, bool show) {
  auto alliance = frc::DriverStation::GetAlliance();
  bool red = alliance && *alliance == frc::DriverStation::Alliance::kRed;
  // test autos become available when tuning mode turns on
  show = show && IsAvailable(number);
  if (number == m_previewNumber && red == m_previewRed && show == m_previewShown) {
    return;
  }
  m_previewNumber = number;
  m_previewRed = red;
  m_previewShown = show;

  std::vector<std::shared_ptr<pathplanner::PathPlannerPath>> paths;
  const AutoOption* codeAuto = FindCodeAuto(number);
  auto pathPlannerName = PathPlannerName(number);
  if (show && codeAuto && codeAuto->path[0] != '\0') {
    paths.push_back(pathplanner::PathPlannerPath::fromPathFile(codeAuto->path));
  } else if (show && pathPlannerName) {
    paths = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile(*pathPlannerName);
  }

  std::vector<frc::Pose2d> poses;
  for (auto path : paths) {
    if (red) {
      path = path->flipPath();
    }
    std::vector<frc::Pose2d> pathPoses = path->getPathPoses();
    poses.insert(poses.end(), pathPoses.begin(), pathPoses.end());
  }
  m_drive.GetField().GetObject("Auto Preview")->SetPoses(poses);
}
