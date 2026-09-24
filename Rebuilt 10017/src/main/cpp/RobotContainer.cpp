//Sam RobotContainer.cpp

#include "RobotContainer.h"
#include "GlobalConstants.h"
#include "subsystems/DriveSubsystem/Constants.h"
//#include "subsystems/IntakeSubsystem/Constants.h"

#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc/MathUtil.h>
#include <units/time.h>

using namespace units::literals;

//Sloppy temporary variables that will moved around later
bool isIntakeDeploy = 0;
//"Hoppers" includes indexer/agitator and kicker/injector
bool isHoppersReverse = 0;
bool isIntakeReverse = 0;
bool isAgitatorForward = 0;


float driveCoefficient = 1;

RobotContainer::RobotContainer()
    : m_autoSelection{
        nt::NetworkTableInstance::GetDefault()
            .GetIntegerTopic("/Auton/Selection")
            .GetEntry(0)  // default = 0 (Shoot Hub)
      }
{
    // ── Publish auton options to SmartDashboard ───────────────
    // SendableChooser still shows in Elastic for reference,
    // but the actual selection comes from the NT integer below.
    m_chooser.SetDefaultOption(kAutoHub,           kAutoHub);
    m_chooser.AddOption(kAutoTower,         kAutoTower);
    m_chooser.AddOption(kAutoBarge,         kAutoBarge);
    m_chooser.AddOption(kAutoCorner,        kAutoCorner);
    m_chooser.AddOption(kAutoNothing,       kAutoNothing);
    m_chooser.AddOption(kAutoRightSide,     kAutoRightSide);
    m_chooser.AddOption(kShootHubRightSide, kShootHubRightSide);
    frc::SmartDashboard::PutData("Auton Chooser", &m_chooser);

    // Publish the auton map so the driver knows which number = which auton
    frc::SmartDashboard::PutString("Auton/0", "Shoot Hub");
    frc::SmartDashboard::PutString("Auton/1", "Shoot Tower");
    frc::SmartDashboard::PutString("Auton/2", "Shoot Barge");
    frc::SmartDashboard::PutString("Auton/3", "Shoot Corner");
    frc::SmartDashboard::PutString("Auton/4", "Right Side");
    frc::SmartDashboard::PutString("Auton/5", "Shoot Hub Right Side");
    frc::SmartDashboard::PutString("Auton/9", "Do Nothing");

    // Set default selection to 0 on startup
    m_autoSelection.Set(0);
    frc::SmartDashboard::PutString("Auton/Selected", "Shoot Hub");

    ConfigureBindings();


//     drive.SetDefaultCommand(frc2::RunCommand(
//     [this] {
//         double speed    = frc::ApplyDeadband( driverController.GetLeftY(),  0.1);
//         double rotation = frc::ApplyDeadband(-driverController.GetRightX(), 0.1);
//         double strafe   = frc::ApplyDeadband( driverController.GetLeftX(),  0.1);

//         drive.ArcadeDrive(speed, rotation);
//         drive.SetHWheelPower(strafe);

//         // ── LT: intake speed proportional to trigger press ────
//         // Fully pressed = full intake speed, half pressed = half speed
//         double ltAxis = driverController.GetLeftTriggerAxis();
//         if (ltAxis > 0.05) {
//             intake.SetIntakePower(ltAxis);  // scales 0.0 → 1.0
//         } else {
//             intake.IntakeOff();
//         }
//     },
//     {&drive, &intake}
// ));
    drive.SetDefaultCommand(frc2::RunCommand(
        [this] {
            double speed    = frc::ApplyDeadband( driverController.GetLeftY(),  0.1);
            double rotation = frc::ApplyDeadband(-driverController.GetRightX(), 0.1);
            //double strafe   = frc::ApplyDeadband( driverController.GetLeftX(),  0.1);
            // if(driverController.frc::XboxController::GetStartButton())
            // drive.ArcadeDrive(driveCoefficient*speed, driveCoefficient*rotation);
            //drive.SetHWheelPower(strafe);
           
            // Have to figure out agitator thing
            // if(agitator.GetAgitatorCurrent() >9999){
                
            //     frc2::cmd::WaitUntil([this]() {
            //         agitator.AgitatorOut();
            //         return (agitator.GetAgitatorCurrent() <=200);
            // }).WithTimeout(0.5_s),
            // frc2::cmd::Run([this]() {
            //     injector.InjectorOut();
            //     agitator.AgitatorAgitate();
        //     }, {&injector, &agitator});

        //     };
        },
        {&drive/*, &agitator*/}
    ));
}

// ─────────────────────────────────────────────────────────────────────────────
//  UpdateDashboard — called every loop from Robot::RobotPeriodic()
//  Shows which auton number is currently selected and its name
// ─────────────────────────────────────────────────────────────────────────────
void RobotContainer::UpdateDashboard() {
    int selected = (int)m_autoSelection.Get();
    frc::SmartDashboard::PutNumber("Auton/SelectionNumber", selected);

    // Show the name of whichever number is selected
    std::string name = "Unknown";
    switch (selected) {
        case 0: name = "Shoot Hub";             break;
        case 1: name = "Shoot Tower";           break;
        case 2: name = "Shoot Barge";           break;
        case 3: name = "Shoot Corner";          break;
        case 4: name = "Right Side";            break;
        case 5: name = "Shoot Hub Right Side";  break;
        case 9: name = "Do Nothing";            break;
    }
    frc::SmartDashboard::PutString("Auton/Selected", name);
}

void RobotContainer::ConfigureBindings() {

    
// There has to be a better way than this, shouldn't have to copy default function to Start event
// But it should work for now
    driverController.Start().OnTrue(frc2::cmd::Run([this]() {
        double speed    = frc::ApplyDeadband( driverController.GetLeftY(),  0.1);
        double rotation = frc::ApplyDeadband(-driverController.GetRightX(), 0.1);

       driveCoefficient = 0.5;
           
        drive.ArcadeDrive(driveCoefficient*speed, driveCoefficient*rotation);
        
    }, {&drive}));

   driverController.Back().OnTrue(frc2::cmd::Run([this]() {
        double speed    = frc::ApplyDeadband( driverController.GetLeftY(),  0.1);
        double rotation = frc::ApplyDeadband(-driverController.GetRightX(), 0.1);

       driveCoefficient = 1;
           
        drive.ArcadeDrive(driveCoefficient*speed, driveCoefficient*rotation);
        
    }, {&drive}));


    driverController.B().OnTrue(frc2::cmd::Run([this]() {
        if(isIntakeDeploy){
        
            intake.Stow();
            isIntakeDeploy = 0;
            isIntakeReverse = 0;
        }
        else{
            
            intake.Deploy();
            isIntakeDeploy = 1;
            isIntakeReverse = 0;
        }
    }, {&intake}));

    driverController.RightTrigger().WhileTrue(
        frc2::cmd::Sequence(
            frc2::cmd::RunOnce([this]() {
                double distance = drive.GetDistanceToHub();
                shooter.SetShooterRPMFromDistance(distance);
            }, {&shooter, &drive}),
            frc2::cmd::WaitUntil([this]() {
                return shooter.AtTargetRPM();
            }).WithTimeout(0.5_s),
            frc2::cmd::Run([this]() {
                injector.InjectorOut();
                agitator.AgitatorAgitate();
            }, {&injector, &agitator})
        )
    );

driverController.Y().OnTrue(frc2::cmd::RunOnce([this]() {
        if(isHoppersReverse){
            agitator.AgitatorOff();
            injector.InjectorOff();

            isHoppersReverse = 0;
            isAgitatorForward = 0;
            
        }
        else{
            agitator.AgitatorOut();
            injector.InjectorIn();
            
            isHoppersReverse = 1;
            isAgitatorForward = 0;
           
            
        }
        }, {&agitator, &injector})

);

driverController.RightBumper().OnTrue(frc2::cmd::RunOnce([this]() {
        if(isAgitatorForward){
            agitator.AgitatorOff();

            isAgitatorForward = 0;
            
        }
        else{
            agitator.AgitatorAgitate();

            isAgitatorForward = 1;


        }
        }, {&agitator})

);

driverController.LeftBumper().OnTrue(frc2::cmd::RunOnce([this]() {
        if(isIntakeReverse){
            intake.IntakeOff();

            isIntakeReverse = 0;
            isIntakeDeploy = 0;
            
        }
        else{
            intake.DeployReverse();

            isIntakeReverse = 1;
            isIntakeDeploy = 0;


        }
        }, {&intake})

);

    driverController.X().OnTrue(frc2::cmd::RunOnce([this]() {
        shooter.ShooterOff();
        injector.InjectorOff();
        agitator.AgitatorOff();
        intake.IntakeOff();
        isIntakeDeploy = 0;
        isIntakeReverse = 0;
        isHoppersReverse = 0;
        isAgitatorForward = 0;
    }, {&shooter, &injector, &agitator, &intake}));

    driverController.POVLeft().WhileTrue(
        frc2::cmd::Sequence(
            frc2::cmd::RunOnce([this]() {
                shooter.ShooterCorner();
            }, {&shooter}),
            frc2::cmd::WaitUntil([this]() {
                return shooter.AtTargetRPM();
            }).WithTimeout(3_s),
            frc2::cmd::Run([this]() {
                injector.InjectorOut();
                agitator.AgitatorAgitate();
            }, {&injector, &agitator})
        )
    );

    driverController.POVRight().WhileTrue(
        frc2::cmd::Sequence(
            frc2::cmd::RunOnce([this]() {
                shooter.ShooterBarge();
            }, {&shooter}),
            frc2::cmd::WaitUntil([this]() {
                return shooter.AtTargetRPM();
            }).WithTimeout(3_s),
            frc2::cmd::Run([this]() {
                injector.InjectorOut();
                agitator.AgitatorAgitate();
            }, {&injector, &agitator})
        )
    );

    driverController.POVUp().WhileTrue(
        frc2::cmd::Sequence(
            frc2::cmd::RunOnce([this]() {
                shooter.ShooterHub();
            }, {&shooter}),
            frc2::cmd::WaitUntil([this]() {
                return shooter.AtTargetRPM();
            }).WithTimeout(3_s),
            frc2::cmd::Run([this]() {
                injector.InjectorOut();
                agitator.AgitatorAgitate();
            }, {&injector, &agitator})
        )
    );

    driverController.POVDown().WhileTrue(
        frc2::cmd::Sequence(
            frc2::cmd::RunOnce([this]() {
                shooter.ShooterTower();
            }, {&shooter}),
            frc2::cmd::WaitUntil([this]() {
                return shooter.AtTargetRPM();
            }).WithTimeout(3_s),
            frc2::cmd::Run([this]() {
                injector.InjectorOut();
                agitator.AgitatorAgitate();
            }, {&injector, &agitator})
        )
    );


    // codriverController.A().OnTrue(frc2::cmd::Run([this]() {
    //     intake.Deploy();
    // }, {&intake}));

    // driverController.Y().OnTrue(frc2::cmd::Run([this](){
    //     intake.IntakeOut();
    //     injector.InjectorOut();
    //     agitator.AgitatorOut();
    // }, {&intake, &agitator, &injector}));

    // driverController.B().WhileTrue(frc2::cmd::Run([this]() {
    //     intake.IntakeOff();
    // }, {&intake}));

    // driverController.RightTrigger().WhileTrue(
    //     frc2::cmd::Sequence(
    //         frc2::cmd::RunOnce([this]() {
    //             shooter.ShooterCorner();
    //         }, {&shooter}),
    //         frc2::cmd::WaitUntil([this]() {
    //             return shooter.AtTargetRPM();
    //         }).WithTimeout(3_s),
    //         frc2::cmd::Run([this]() {
    //             injector.InjectorOut();
    //             agitator.AgitatorAgitate();
    //         }, {&injector, &agitator})
    //     )
    // );


    // codriverController.Y().OnTrue(
//     frc2::cmd::Sequence(
//         frc2::cmd::RunOnce([this]() {
//             intake.Stow();
//         }, {&intake}),
//         frc2::cmd::Wait(1_s),
//         frc2::cmd::RunOnce([this]() {
//             intake.Deploy();
//         }, {&intake})
//     )
// );

    // codriverController.X().OnTrue(frc2::cmd::RunOnce([this]() {
    //     shooter.ShooterOff();
    //     injector.InjectorOff();
    //     agitator.AgitatorOff();
    // }, {&shooter, &injector, &agitator}));

    //     codriverController.X().OnTrue(frc2::cmd::RunOnce([this]() {
    //     shooter.ShooterOff();
    //     injector.InjectorOff();
    //     agitator.AgitatorOff();
    // }, {&shooter, &injector, &agitator}));

    // driverController.Back().OnTrue(frc2::cmd::RunOnce([this]() {
    //     drive.ResetOdometryToKnownPosition();
    // }, {&drive}));

   
    // An originial POV just in case
    // driverController.POVUp().WhileTrue(frc2::cmd::Run([this]() {
    //     shooter.ShooterHub();
    //     injector.InjectorIn();
    //     agitator.AgitatorIn();
    // }, {&shooter, &injector, &agitator}));

    // ── LB: intake arm toggle (deploy ↔ stow) ────────────────
//     driverController.LeftBumper().OnTrue(frc2::cmd::RunOnce([this]() {
//     double currentAngle = intake.GetArmAngle();
//     double midpoint = (IntakeConstants::kStowedAngle + IntakeConstants::kDeployAngle) / 2.0;

//     if (currentAngle < midpoint) {
//         intake.Deploy();
//     } else {
//         intake.Stow();
//     }
// }, {&intake}));

    // codriverController.POVLeft().WhileTrue(frc2::cmd::Run([this]() {
    //     shooter.ShooterCorner();
    //     injector.InjectorIn();
    //     agitator.AgitatorIn();
    // }, {&shooter, &injector, &agitator}));

    // codriverController.POVRight().WhileTrue(frc2::cmd::Run([this]() {
    //     shooter.ShooterBarge();
    //     injector.InjectorIn();
    //     agitator.AgitatorIn();
    // }, {&shooter, &injector, &agitator}));

    // codriverController.POVUp().WhileTrue(frc2::cmd::Run([this]() {
    //     shooter.ShooterHub();
    //     injector.InjectorIn();
    //     agitator.AgitatorIn();
    // }, {&shooter, &injector, &agitator}));

    // codriverController.POVDown().WhileTrue(frc2::cmd::Run([this]() {
    //     shooter.ShooterTower();
    //     injector.InjectorIn();
    //     agitator.AgitatorIn();
    // }, {&shooter, &injector, &agitator}));
}

// ─────────────────────────────────────────────────────────────────────────────
//  GetAutonomousCommand
//  Reads the integer from NetworkTables written by Elastic
// ─────────────────────────────────────────────────────────────────────────────
std::optional<frc2::CommandPtr> RobotContainer::GetAutonomousCommand() {
    int selected = (int)m_autoSelection.Get();

    switch (selected) {
        case 0: return ShootHubAuto     (&shooter, &injector, &agitator, &intake);
        case 1: return ShootTowerAuto   (&shooter, &injector, &agitator, &intake);
        case 2: return ShootBargeAuto   (&shooter, &injector, &agitator, &intake);
        case 3: return ShootCornerAuto  (&shooter, &injector, &agitator, &intake);
        case 4: return RightSideAuto    (&shooter, &injector, &agitator, &intake);
        case 5: return ShootHubRightAuto(&shooter, &injector, &agitator, &intake);
        default: return std::nullopt;  // 9 or anything else = do nothing
    }
}