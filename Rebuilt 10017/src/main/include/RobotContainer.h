//Sam RobotContainer.h

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/IntegerTopic.h>

#include "subsystems/Drivesubsystem/DriveSubsystem.h"
#include "subsystems/ShooterSubsystem/ShooterSubsystem.h"
#include "subsystems/InjectorSubsystem/InjectorSubsystem.h"
#include "subsystems/LinearActuatorSubsystem/LinearActuatorSubsystem.h"
#include "subsystems/IntakeSubsystem/IntakeSubsystem.h"
#include "subsystems/AgitatorSubsystem/AgitatorSubsystem.h"
#include "autos/ShootHubAuto.h"
#include "autos/ShootTowerAuto.h"
#include "autos/ShootBargeAuto.h"
#include "autos/ShootCornerAuto.h"
#include "autos/RightSideAuto.h"
#include "autos/ShootHubRightAuto.h"

class RobotContainer {
public:
    RobotContainer();

    std::optional<frc2::CommandPtr> GetAutonomousCommand();
    void UpdateDashboard();

private:
    DriveSubsystem          drive;
    ShooterSubsystem        shooter{};
    InjectorSubsystem       injector{};
    LinearActuatorSubsystem actuator{};
    IntakeSubsystem         intake{};
    AgitatorSubsystem       agitator{};

    frc2::CommandXboxController driverController{0};
    frc2::CommandXboxController codriverController{1};

    bool m_autoRPMMode = false;

    // Auton numbers:
    //   0 = Shoot Hub        (default)
    //   1 = Shoot Tower
    //   2 = Shoot Barge
    //   3 = Shoot Corner
    //   4 = Right Side
    //   5 = Shoot Hub Right Side
    //   9 = Do Nothing
    nt::IntegerEntry m_autoSelection;

    // Still keep the SendableChooser for display purposes
    frc::SendableChooser<std::string> m_chooser;
    static constexpr const char* kAutoHub           = "Shoot Hub";
    static constexpr const char* kAutoTower         = "Shoot Tower";
    static constexpr const char* kAutoBarge         = "Shoot Barge";
    static constexpr const char* kAutoCorner        = "Shoot Corner";
    static constexpr const char* kAutoNothing       = "Do Nothing";
    static constexpr const char* kAutoRightSide     = "Right Side";
    static constexpr const char* kShootHubRightSide = "Shoot Hub Right Side";

    void ConfigureBindings();
};