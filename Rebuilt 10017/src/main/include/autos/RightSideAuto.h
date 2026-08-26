#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <units/time.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include "subsystems/ShooterSubsystem/ShooterSubsystem.h"
#include "subsystems/InjectorSubsystem/InjectorSubsystem.h"
#include "subsystems/AgitatorSubsystem/AgitatorSubsystem.h"
#include "subsystems/IntakeSubsystem/IntakeSubsystem.h"

using namespace units::literals;

inline frc2::CommandPtr RightSideAuto(
    ShooterSubsystem*  shooter,
    InjectorSubsystem* injector,
    AgitatorSubsystem* agitator,
    IntakeSubsystem*   intake)
{
    return frc2::cmd::Sequence(

        // ── Step 1: First shoot — Hub RPM ─────────────────────
        frc2::cmd::RunOnce([shooter, injector]() {
            shooter->ShooterHub();
            injector->InjectorIn();
        }, {shooter, injector}),

        frc2::cmd::WaitUntil([shooter]() {
            return shooter->AtTargetRPM();
        }).WithTimeout(3_s),

        // Feed balls for 3 seconds
        frc2::cmd::RunOnce([agitator]() {
            agitator->AgitatorIn();
        }, {agitator})
        .AndThen(frc2::cmd::Wait(5_s)),

        // Stop shooter and feeder before driving
        frc2::cmd::RunOnce([shooter, injector, agitator]() {
            shooter->ShooterOff();
            injector->InjectorOff();
            agitator->AgitatorOff();
        }, {shooter, injector, agitator}),

        // ── Step 2: Drive path to collect balls ───────────────
        // Run intake at the same time as driving so balls are
        // collected as soon as the robot reaches them
        frc2::cmd::Parallel(
            pathplanner::PathPlannerAuto("Right Side").ToPtr(),
            frc2::cmd::Run([intake]() {
                intake->Deploy();
            }, {intake})
        ),

        // ── Step 3: Stop intake after path ────────────────────
        frc2::cmd::RunOnce([intake]() {
            intake->Stow();
            intake->IntakeOff();
        }, {intake}),

        // ── Step 4: Spin up to Corner RPM while driving back ──
        // Starts spinning the shooter early so it's up to speed
        // by the time the robot reaches the shooting position
        frc2::cmd::RunOnce([shooter]() {
            shooter->ShooterCorner();
        }, {shooter}),

        // Wait for shooter to reach Corner RPM (4s max)
        frc2::cmd::WaitUntil([shooter]() {
            return shooter->AtTargetRPM();
        }).WithTimeout(4_s),

        // ── Step 5: Second shoot — Corner RPM ─────────────────
        frc2::cmd::RunOnce([injector, agitator]() {
            injector->InjectorOut();
            agitator->AgitatorAgitate();
        }, {injector, agitator})
        .AndThen(frc2::cmd::Wait(4_s)),

        // ── Step 6: Stop everything ───────────────────────────
        frc2::cmd::RunOnce([shooter, injector, agitator]() {
            shooter->ShooterOff();
            injector->InjectorOff();
            agitator->AgitatorOff();
        }, {shooter, injector, agitator})
    );
}