//DOESN'T WORK





//DOESN'T WORK



#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Servo.h>
#include <units/length.h>
#include "constants.h"

/**
 * LinearActuatorSubsystem
 *
 *
 * Usage (in RobotContainer or a Command):
 *   // Extend to 75 mm
 *   m_actuator.SetPosition(75.0_mm);
 *
 *   // Extend fully
 *   m_actuator.SetExtended();
 *
 *   // Retract fully
 *   m_actuator.SetRetracted();
 */
class LinearActuatorSubsystem : public frc2::SubsystemBase {
public:
    LinearActuatorSubsystem();

    // ── Primary control ──────────────────────────────────────

    /**
     * Command the actuator to a target position.
     * @param targetPosition_mm  Desired position in mm [0, 100].
     *                           Values are clamped to the valid range.
     */
    void SetPosition(double targetPosition_mm);

    /** Convenience: fully retract (0 mm). */
    void SetRetracted();

    /** Convenience: fully extend (100 mm). */
    void SetExtended();

    /**
     * Set position via a normalised value.
     * @param value  0.0 = fully retracted, 1.0 = fully extended.
     */
    void SetNormalized(double value);

    // ── Telemetry ────────────────────────────────────────────

    /**
     * Returns the last commanded position in mm.
     * Note: the L16-R has no feedback; this is open-loop.
     */
    double GetCommandedPosition_mm() const;

    /** Returns the last commanded normalised value [0.0, 1.0]. */
    double GetNormalizedPosition()   const;

    // ── SubsystemBase override ────────────────────────────────
    void Periodic() override;

private:
    frc::Servo m_servo;

    double m_commandedPosition_mm { 0.0 };

    /** Convert mm → normalised [0, 1] for WPILib Servo. */
    static double MmToNormalized(double mm);

    /** Clamp a value to [lo, hi]. */
    static double Clamp(double value, double lo, double hi);
};