//DOESN'T WORK





//DOESN'T WORK
#include "subsystems/LinearActuatorSubsystem/LinearActuatorSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <units/time.h>
#include <algorithm>

using namespace ActuonixL16;
using namespace units::literals; // enables the _us literal suffix

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────────────────────
LinearActuatorSubsystem::LinearActuatorSubsystem()
    : m_servo(kPwmChannel)
{
    // SetBounds requires units::microsecond_t in WPILib 2026.
    // Literal values used directly so this compiles regardless of
    // whether constants.h defines the pulse-width consts as double.
    m_servo.SetBounds(
        2000_us,   // max          – full extension
        1520_us,   // deadbandMax
        1500_us,   // centre
        1480_us,   // deadbandMin
        1000_us    // min          – full retraction
    );

    // Start fully retracted
    SetRetracted();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Primary control
// ─────────────────────────────────────────────────────────────────────────────
void LinearActuatorSubsystem::SetPosition(double targetPosition_mm)
{
    double clamped = Clamp(targetPosition_mm,
                           kMinPosition_mm,
                           kMaxPosition_mm);

    m_commandedPosition_mm = clamped;
    m_servo.Set(MmToNormalized(clamped));
}

void LinearActuatorSubsystem::SetRetracted()
{
    SetPosition(kMinPosition_mm);
}

void LinearActuatorSubsystem::SetExtended()
{
    SetPosition(kMaxPosition_mm);
}

void LinearActuatorSubsystem::SetNormalized(double value)
{
    double clamped = Clamp(value, 0.0, 1.0);
    SetPosition(clamped * kStroke_mm);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Telemetry
// ─────────────────────────────────────────────────────────────────────────────
double LinearActuatorSubsystem::GetCommandedPosition_mm() const
{
    return m_commandedPosition_mm;
}

double LinearActuatorSubsystem::GetNormalizedPosition() const
{
    return MmToNormalized(m_commandedPosition_mm);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Periodic – runs every 20 ms
// ─────────────────────────────────────────────────────────────────────────────
void LinearActuatorSubsystem::Periodic()
{
    frc::SmartDashboard::PutNumber(
        "Actuator/CommandedPosition_mm", m_commandedPosition_mm);
    frc::SmartDashboard::PutNumber(
        "Actuator/NormalizedPosition",   GetNormalizedPosition());
}

// ─────────────────────────────────────────────────────────────────────────────
//  Private helpers
// ─────────────────────────────────────────────────────────────────────────────
double LinearActuatorSubsystem::MmToNormalized(double mm)
{
    if (kStroke_mm <= 0.0) return 0.0;
    return mm / kStroke_mm;
}

double LinearActuatorSubsystem::Clamp(double value, double lo, double hi)
{
    return std::clamp(value, lo, hi);
}