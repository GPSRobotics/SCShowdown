#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <units/time.h>

#include "subsystems/ShooterSubsystem/ShooterSubsystem.h"
#include "subsystems/InjectorSubsystem/InjectorSubsystem.h"
#include "subsystems/AgitatorSubsystem/AgitatorSubsystem.h"
#include "subsystems/IntakeSubsystem/IntakeSubsystem.h"

using namespace units::literals;

// ── ShootAuto ─────────────────────────────────────────────────────────────────
// Shoots 8 balls from starting position:
//   1. Spin up to Hub RPM
//   2. Wait for speed (3s max)
//   3. Stow intake arm to feed balls toward shooter
//   4. Run agitator + injector for 20 seconds
//   5. Stop everything
// ─────────────────────────────────────────────────────────────────────────────

inline frc2::CommandPtr ShootHubAuto(
    ShooterSubsystem*  shooter,
    InjectorSubsystem* injector,
    AgitatorSubsystem* agitator,
    IntakeSubsystem*   intake)
{
    return frc2::cmd::Sequence(

        // Step 1: spin up shooter to Hub RPM
        frc2::cmd::RunOnce([shooter, injector]() {
            shooter->ShooterHub();
            injector->InjectorIn();
        }, {shooter, injector}),

        // Step 2: wait until flywheel reaches speed (3s timeout)
        frc2::cmd::WaitUntil([shooter]() {
            return shooter->AtTargetRPM();
        }).WithTimeout(3_s),

        // Step 3: stow intake arm so balls flow toward hopper
        frc2::cmd::RunOnce([intake]() {
            intake->DeployReverse();
        }, {intake}),

        // Step 4: run agitator + injector for 20 seconds
        frc2::cmd::RunOnce([agitator]() {
            agitator->AgitatorIn();
        }, {agitator})
        .AndThen(frc2::cmd::Wait(10_s)),

        // Step 5: stop everything
        frc2::cmd::RunOnce([shooter, injector, agitator, intake]() {
            shooter->ShooterOff();
            injector->InjectorOff();
            agitator->AgitatorOff();
            intake->IntakeOff();
        }, {shooter, injector, agitator, intake})

    );
}