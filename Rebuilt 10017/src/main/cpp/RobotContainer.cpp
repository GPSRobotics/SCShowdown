#include "RobotContainer.h"

#include <algorithm>

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

#include "controls/ControlsXbox.h"
#include "controls/ControlsXboxSim.h"
#include "subsystems/drive/DriveCommands.h"
#include "subsystems/shooter/ShooterCommands.h"

RobotContainer::RobotContainer() { ConfigureBindings(); }

void RobotContainer::ConfigureBindings() {
  m_drive.SetDefaultCommand(DriveCommands::JoystickDrive(
      m_drive, [this] { return m_controls->GetForward(); },
      [this] { return m_controls->GetTurn(); }));
  m_controls->ToggleSlowMode().OnTrue(m_drive.ToggleSpeedModeCommand(Drive::SpeedMode::kSlow));
  m_controls->ToggleTurnOnly().OnTrue(m_drive.ToggleSpeedModeCommand(Drive::SpeedMode::kTurnOnly));

  m_controls->ToggleArm().OnTrue(m_intake.ToggleArmCommand());
  m_controls->ToggleIntake().OnTrue(m_intake.ToggleIntakeCommand());
  m_controls->ToggleEject().OnTrue(m_intake.ToggleEjectCommand());
  m_controls->ToggleAgitator().OnTrue(m_agitator.ToggleFeedCommand());
  m_controls->ToggleUnjam().OnTrue(ShooterCommands::ToggleUnjam(m_injector, m_agitator));
  m_controls->StopAll().OnTrue(frc2::cmd::RunOnce(
      [this] { StopMechanisms(); }, {&m_shooter, &m_injector, &m_agitator, &m_intake}));

  m_controls->ToggleFullPowerShot().ToggleOnTrue(
      ShooterCommands::FullPowerShot(m_shooter, m_injector, m_agitator));
  m_controls->ChargeHub().OnTrue(Charge(Shooter::Location::kHub));
  m_controls->ChargeTrench().OnTrue(Charge(Shooter::Location::kTrench));
  m_controls->ChargeCorner().OnTrue(Charge(Shooter::Location::kCorner));
  m_controls->ChargeTower().OnTrue(Charge(Shooter::Location::kTower));
}

frc2::CommandPtr RobotContainer::Charge(Shooter::Location location) {
  return ShooterCommands::ToggleCharge(m_shooter, m_injector, location);
}

std::optional<frc2::CommandPtr> RobotContainer::GetAutonomousCommand() {
  return m_autoChooser.GetSelected();
}

void RobotContainer::Periodic() {
  m_autoChooser.Periodic();
  frc::SmartDashboard::PutString("Shot/Status", ShotStatus());
  // the driver station sends -1 when theres no match timer (not practice mode or fms)
  frc::SmartDashboard::PutNumber("Match Time",
                                 std::max(0.0, frc::DriverStation::GetMatchTime().value()));
  frc::SmartDashboard::PutNumber("Battery Voltage",
                                 frc::RobotController::GetBatteryVoltage().value());
}

void RobotContainer::StopMechanisms() {
  m_shooter.Stop();
  m_injector.Stop();
  m_agitator.Stop();
  m_intake.StopRollers();
}

// plain words so the driver knows what to press next
std::string RobotContainer::ShotStatus() const {
  if (m_injector.IsGoal(Injector::Goal::kReverse) || m_agitator.IsGoal(Agitator::Goal::kReverse)) {
    return "Unjamming - press Y again to stop";
  }

  bool hasSpot = m_shooter.GetLocation() != Shooter::Location::kNone;
  std::string spot = m_shooter.GetLocationName();
  bool feeding = m_agitator.IsGoal(Agitator::Goal::kFeed);

  switch (m_shooter.GetState()) {
    case Shooter::State::kDisabled:
      return "Robot disabled";
    case Shooter::State::kFullPower:
      return m_injector.IsGoal(Injector::Goal::kFeed) ? "Full power shot - feeding"
                                                      : "Full power shot - spinning up";
    case Shooter::State::kSpinningUp:
      return "Charging " + (hasSpot ? spot + " " : "") + "(" +
             std::to_string(static_cast<int>(m_shooter.GetAverageRpm())) + " / " +
             std::to_string(static_cast<int>(m_shooter.GetTargetRpm())) + " RPM)";
    case Shooter::State::kAtSpeed:
      if (feeding) {
        return hasSpot ? "Shooting from " + spot : "Shooting";
      }
      return hasSpot ? spot + " ready - press RB to shoot" : "At speed";
    case Shooter::State::kIdle:
      return feeding ? "Belts on, shooter off" : "Shooter off - pick a spot on the D-pad";
  }
  return "";
}

std::unique_ptr<Controls> RobotContainer::CreateControls() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<ControlsXboxSim>(0);
  }
  return std::make_unique<ControlsXbox>(0);
}
