#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <units/time.h>

#include "subsystems/ShooterSubsystem/ShooterSubsystem.h"
#include "subsystems/InjectorSubsystem/InjectorSubsystem.h"
#include "subsystems/AgitatorSubsystem/AgitatorSubsystem.h"
#include "subsystems/IntakeSubsystem/IntakeSubsystem.h"

using namespace units::literals;

inline frc2::CommandPtr ShootHubRightAuto(
    ShooterSubsystem*  shooter,
    InjectorSubsystem* injector,
    AgitatorSubsystem* agitator,
    IntakeSubsystem*   intake)
{
    return frc2::cmd::Sequence(

        frc2::cmd::RunOnce([shooter, injector]() {
            shooter->ShooterHub();
            injector->InjectorIn();
        }, {shooter, injector}),

        frc2::cmd::WaitUntil([shooter]() {
            return shooter->AtTargetRPM();
        }).WithTimeout(3_s),

        frc2::cmd::RunOnce([intake]() {
            intake->DeployReverse();
        }, {intake}),

        frc2::cmd::RunOnce([agitator]() {
            agitator->AgitatorIn();
        }, {agitator})
        .AndThen(frc2::cmd::Wait(10_s)),

        frc2::cmd::RunOnce([shooter, injector, agitator, intake]() {
            shooter->ShooterOff();
            injector->InjectorOff();
            agitator->AgitatorOff();
            intake->IntakeOff();
        }, {shooter, injector, agitator, intake})
    );
}