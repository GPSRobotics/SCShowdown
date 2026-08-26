#include "subsystems/ShooterSubsystem/ShooterSubsystem.h"
#include "rev/config/SparkFlexConfig.h"

#include <iostream>
#include <cmath>
#include <algorithm>

using namespace frc;
using namespace rev::spark;

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────────────────────
ShooterSubsystem::ShooterSubsystem()
    : leftShooterMotor  { ShooterConstants::kLeftShooterPort,  SparkLowLevel::MotorType::kBrushless }
    , rightShooterMotor { ShooterConstants::kRightShooterPort, SparkLowLevel::MotorType::kBrushless }
    , leftPID           { leftShooterMotor.GetClosedLoopController()  }
    , rightPID          { rightShooterMotor.GetClosedLoopController() }
    , leftEncoder       { leftShooterMotor.GetEncoder()  }
    , rightEncoder      { rightShooterMotor.GetEncoder() }
{
    // ── Left motor ────────────────────────────────────────────
    SparkFlexConfig leftConfig{};
    leftConfig.Inverted(false)
              .SetIdleMode(SparkFlexConfig::IdleMode::kCoast);
    leftConfig.closedLoop
        .Pidf(ShooterConstants::kP,  ShooterConstants::kI,
              ShooterConstants::kD,  ShooterConstants::kFF)
        .OutputRange(ShooterConstants::kMinOutput, ShooterConstants::kMaxOutput);
    leftShooterMotor.Configure(leftConfig,
        rev::ResetMode::kResetSafeParameters,
        rev::PersistMode::kPersistParameters);

    // ── Right motor (inverted) ────────────────────────────────
    SparkFlexConfig rightConfig{};
    rightConfig.Inverted(true)
               .SetIdleMode(SparkFlexConfig::IdleMode::kCoast);
    rightConfig.closedLoop
        .Pidf(ShooterConstants::kP,  ShooterConstants::kI,
              ShooterConstants::kD,  ShooterConstants::kFF)
        .OutputRange(ShooterConstants::kMinOutput, ShooterConstants::kMaxOutput);
    rightShooterMotor.Configure(rightConfig,
        rev::ResetMode::kResetSafeParameters,
        rev::PersistMode::kPersistParameters);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Periodic
// ─────────────────────────────────────────────────────────────────────────────
void ShooterSubsystem::Periodic()
{
    frc::SmartDashboard::PutNumber("Shooter/LeftRPM",    GetLeftRPM());
    frc::SmartDashboard::PutNumber("Shooter/RightRPM",   GetRightRPM());
    frc::SmartDashboard::PutNumber("Shooter/TargetRPM",  targetRPM);
    frc::SmartDashboard::PutBoolean("Shooter/AtSpeed",   AtTargetRPM());
}

// ─────────────────────────────────────────────────────────────────────────────
//  Fixed RPM presets — unchanged
// ─────────────────────────────────────────────────────────────────────────────
void ShooterSubsystem::SetShooterRPM(double rpm)
{
    targetRPM = rpm;
    leftPID .SetReference(rpm, SparkLowLevel::ControlType::kVelocity);
    rightPID.SetReference(rpm, SparkLowLevel::ControlType::kVelocity);
}

void ShooterSubsystem::ShooterCorner()         { SetShooterRPM(ShooterConstants::kShootCornerRPM); }
void ShooterSubsystem::ShooterOff()            { SetShooterRPM(ShooterConstants::kStopRPM);        }
void ShooterSubsystem::ShooterIntake()         { SetShooterRPM(ShooterConstants::kIntakeRPM);      }
void ShooterSubsystem::ShooterBarge()          { SetShooterRPM(ShooterConstants::kShootBargeRPM);  }
void ShooterSubsystem::ShooterTower()          { SetShooterRPM(ShooterConstants::kShootTowerRPM);  }
void ShooterSubsystem::ShooterHub()            { SetShooterRPM(ShooterConstants::kShootHubRPM);    }
void ShooterSubsystem::ShooterHubRight()     { SetShooterRPM(ShooterConstants::kShootHubRightRPM);    }

// ─────────────────────────────────────────────────────────────────────────────
//  Distance-based RPM
// ─────────────────────────────────────────────────────────────────────────────
double ShooterSubsystem::SetShooterRPMFromDistance(double distanceMeters)
{
    double rpm = InterpolateRPM(distanceMeters);
    SetShooterRPM(rpm);
    frc::SmartDashboard::PutNumber("Shooter/DistanceInput_m", distanceMeters);
    frc::SmartDashboard::PutNumber("Shooter/InterpolatedRPM", rpm);
    return rpm;
}

// ─────────────────────────────────────────────────────────────────────────────
//  InterpolateRPM (static)
//  Linearly interpolates between kRpmTable entries.
//  Clamps to the nearest table entry if out of range.
// ─────────────────────────────────────────────────────────────────────────────
double ShooterSubsystem::InterpolateRPM(double distanceMeters)
{
    const auto& table = ShooterConstants::kRpmTable;

    // Below the minimum table distance — use the first (closest) RPM
    if (distanceMeters <= table.front().first) {
        return table.front().second;
    }

    // Above the maximum table distance — use the last (furthest) RPM
    if (distanceMeters >= table.back().first) {
        return table.back().second;
    }

    // Find the two surrounding entries and interpolate
    for (size_t i = 0; i + 1 < table.size(); ++i) {
        double d0   = table[i].first,       rpm0 = table[i].second;
        double d1   = table[i + 1].first,   rpm1 = table[i + 1].second;

        if (distanceMeters >= d0 && distanceMeters <= d1) {
            // t = 0.0 at d0, t = 1.0 at d1
            double t = (distanceMeters - d0) / (d1 - d0);
            return rpm0 + t * (rpm1 - rpm0);
        }
    }

    return table.front().second;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Telemetry
// ─────────────────────────────────────────────────────────────────────────────
double ShooterSubsystem::GetLeftRPM()  const { return leftEncoder.GetVelocity();  }
double ShooterSubsystem::GetRightRPM() const { return rightEncoder.GetVelocity(); }

bool ShooterSubsystem::AtTargetRPM() const
{
    bool leftOk  = std::abs(GetLeftRPM()  - targetRPM) < ShooterConstants::kRPMTolerance;
    bool rightOk = std::abs(GetRightRPM() - targetRPM) < ShooterConstants::kRPMTolerance;
    return leftOk && rightOk;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Legacy stubs 
// ─────────────────────────────────────────────────────────────────────────────
void ShooterSubsystem::SetShooterPower(double newPower) {}
void ShooterSubsystem::GetShooterPower() {}
void ShooterSubsystem::SetShooterState(int newState)    { ShooterState = newState; }
int  ShooterSubsystem::GetShooterStates()               { return ShooterState; }
void ShooterSubsystem::SetShooterBrakeMode(bool state)  {}
void ShooterSubsystem::ConfigShooter()                  {}
void ShooterSubsystem::ShooterIn()                      { SetShooterRPM(ShooterConstants::kReverseRPM); }