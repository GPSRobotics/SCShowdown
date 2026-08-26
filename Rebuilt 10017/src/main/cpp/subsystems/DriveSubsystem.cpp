#include "subsystems/DriveSubsystem/DriveSubsystem.h"
#include "subsystems/DriveSubsystem/Constants.h"

#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <rev/config/SparkMaxConfig.h>
#include <frc/DriverStation.h>
#include <algorithm>
#include <cmath>

using namespace ctre::phoenix6;
using namespace DriveConstants;

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────────────────────
DriveSubsystem::DriveSubsystem()
    : leftMotor  { kLeftMotorID  }
    , rightMotor { kRightMotorID }
    , middleMotor{ kMiddleMotorID, rev::spark::SparkLowLevel::MotorType::kBrushless }
{
    // ── Left TalonFX ──────────────────────────────────────────
    configs::TalonFXConfiguration leftConfig{};
    leftConfig.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
    leftMotor.GetConfigurator().Apply(leftConfig);

    // ── Right TalonFX (inverted) ──────────────────────────────
    configs::TalonFXConfiguration rightConfig{};
    rightConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
    rightMotor.GetConfigurator().Apply(rightConfig);

    // ── H-wheel SparkMax (NEO) ────────────────────────────────
    rev::spark::SparkMaxConfig middleConfig{};
    middleConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    middleConfig.Inverted(false);
    middleMotor.Configure(middleConfig,
        rev::ResetMode::kResetSafeParameters,
        rev::PersistMode::kPersistParameters);

    // ── Zero drive encoders on startup ────────────────────────
    leftMotor.SetPosition(0_tr);
    rightMotor.SetPosition(0_tr);

    // ── NavX reset ────────────────────────────────────────────
    m_gyro.ZeroYaw();

    // ── Seed odometry at known start position ─────────────────
    frc::Pose2d startPose{
        units::meter_t{ kResetX_m },
        units::meter_t{ kResetY_m },
        frc::Rotation2d{ units::degree_t{ kResetAngle_deg } }
    };
    m_odometry.ResetPosition(
        GetHeading(),
        GetLeftDistanceMeters(),
        GetRightDistanceMeters(),
        startPose);

    // ── PathPlanner AutoBuilder configuration ─────────────────
    // Tells PathPlanner how to read and control your robot.
    // PPLTVController is designed for differential drive robots.
    pathplanner::AutoBuilder::configure(
        [this]() { return GetPose(); },

        [this](frc::Pose2d pose) { ResetOdometry(pose); },

        [this]() { return GetChassisSpeeds(); },

        [this](frc::ChassisSpeeds speeds) { DriveWithSpeeds(speeds); },

        std::make_shared<pathplanner::PPLTVController>(0.02_s),

        pathplanner::RobotConfig::fromGUISettings(),

        []() {
            // Flip the path if we're on Red alliance so the same
            // path works from both sides of the field
            auto alliance = frc::DriverStation::GetAlliance();
            return alliance && *alliance == frc::DriverStation::Alliance::kRed;
        },
        this
    );
}

void DriveSubsystem::Periodic()
{
    leftPosition.Refresh();
    rightPosition.Refresh();
    leftVelocity.Refresh();
    rightVelocity.Refresh();

    m_odometry.Update(GetHeading(), GetLeftDistanceMeters(), GetRightDistanceMeters());

    frc::Pose2d pose = GetPose();
    frc::SmartDashboard::PutNumber("Drive/LeftEncoder",    leftPosition.GetValue().value());
    frc::SmartDashboard::PutNumber("Drive/RightEncoder",   rightPosition.GetValue().value());
    frc::SmartDashboard::PutNumber("Drive/HWheelPower",    middleMotor.Get());
    frc::SmartDashboard::PutNumber("Odometry/X_m",         pose.X().value());
    frc::SmartDashboard::PutNumber("Odometry/Y_m",         pose.Y().value());
    frc::SmartDashboard::PutNumber("Odometry/Angle_deg",   pose.Rotation().Degrees().value());
    frc::SmartDashboard::PutNumber("Gyro/Heading_deg",     m_gyro.GetAngle());
    frc::SmartDashboard::PutNumber("Odometry/DistToHub_m", GetDistanceToHub());
}

void DriveSubsystem::ArcadeDrive(double speed, double rotation)
{
    double leftOutput  = std::clamp(speed + rotation, -1.0, 1.0);
    double rightOutput = std::clamp(speed - rotation, -1.0, 1.0);

    controls::DutyCycleOut leftControl  { leftOutput  };
    controls::DutyCycleOut rightControl { rightOutput };

    leftMotor.SetControl(leftControl);
    rightMotor.SetControl(rightControl);
}

void DriveSubsystem::SetHWheelPower(double power)
{
    middleMotor.Set(std::clamp(power, -1.0, 1.0));
}

// Called by PathPlanner every loop tick while following a path.
// Converts ChassisSpeeds (vx, vy, omega) into left/right motor outputs.
void DriveSubsystem::DriveWithSpeeds(frc::ChassisSpeeds speeds)
{
    auto wheelSpeeds = m_kinematics.ToWheelSpeeds(speeds);
    wheelSpeeds.Desaturate(units::meters_per_second_t{ 4.5 }); // clamp to max speed

    // Convert m/s to duty cycle [-1, 1]
    // kMaxSpeedMetersPerSecond should match your robot's actual top speed
    double leftOutput  = wheelSpeeds.left.value()  / kMaxSpeedMetersPerSecond;
    double rightOutput = wheelSpeeds.right.value()  / kMaxSpeedMetersPerSecond;

    controls::DutyCycleOut leftControl  { std::clamp(leftOutput,  -1.0, 1.0) };
    controls::DutyCycleOut rightControl { std::clamp(rightOutput, -1.0, 1.0) };

    leftMotor.SetControl(leftControl);
    rightMotor.SetControl(rightControl);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Pose / Odometry
// ─────────────────────────────────────────────────────────────────────────────
frc::Pose2d DriveSubsystem::GetPose() const
{
    return m_odometry.GetPose();
}

frc::ChassisSpeeds DriveSubsystem::GetChassisSpeeds() const
{
    double leftMPS  = leftVelocity.GetValue().value()  * kMetersPerRotation;
    double rightMPS = rightVelocity.GetValue().value() * kMetersPerRotation;

    frc::DifferentialDriveWheelSpeeds wheelSpeeds{
        units::meters_per_second_t{ leftMPS },
        units::meters_per_second_t{ rightMPS }
    };

    return m_kinematics.ToChassisSpeeds(wheelSpeeds);
}

double DriveSubsystem::GetDistanceToHub() const
{
    frc::Translation2d robotPos = GetPose().Translation();

    double hubX = kBlueHubX_m;
    double hubY = kBlueHubY_m;

    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance && *alliance == frc::DriverStation::Alliance::kRed) {
        hubX = kRedHubX_m;
        hubY = kRedHubY_m;
    }

    double dx = robotPos.X().value() - hubX;
    double dy = robotPos.Y().value() - hubY;
    return std::sqrt(dx * dx + dy * dy);
}

void DriveSubsystem::ResetOdometryToKnownPosition()
{
    frc::Pose2d knownPose{
        units::meter_t{ kResetX_m },
        units::meter_t{ kResetY_m },
        frc::Rotation2d{ units::degree_t{ kResetAngle_deg } }
    };
    ResetOdometry(knownPose);
}

void DriveSubsystem::ResetOdometry(const frc::Pose2d& pose)
{
    leftMotor.SetPosition(0_tr);
    rightMotor.SetPosition(0_tr);
    m_gyro.ZeroYaw();

    m_odometry.ResetPosition(
        frc::Rotation2d{ units::degree_t{ pose.Rotation().Degrees().value() } },
        0_m, 0_m,
        pose);

    frc::SmartDashboard::PutString("Odometry/Status", "Reset to known position!");
}

units::meter_t DriveSubsystem::GetLeftDistanceMeters() const
{
    return units::meter_t{ leftPosition.GetValue().value() * kMetersPerRotation };
}

units::meter_t DriveSubsystem::GetRightDistanceMeters() const
{
    return units::meter_t{ rightPosition.GetValue().value() * kMetersPerRotation };
}

frc::Rotation2d DriveSubsystem::GetHeading()
{
    return frc::Rotation2d{ units::degree_t{ -m_gyro.GetAngle() } };
}