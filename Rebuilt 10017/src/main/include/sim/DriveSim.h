#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/system/plant/DCMotor.h>
#include <hal/SimDevice.h>

#include "subsystems/drive/DriveConstants.h"

// drivetrain sim, writes wheel positions and heading back to the krakens and navx
class DriveSim {
 public:
  DriveSim(ctre::phoenix6::hardware::TalonFX& leftMotor,
           ctre::phoenix6::hardware::TalonFX& rightMotor);

  void Update();

 private:
  void FindGyro();

  ctre::phoenix6::sim::TalonFXSimState& m_leftSim;
  ctre::phoenix6::sim::TalonFXSimState& m_rightSim;

  frc::sim::DifferentialDrivetrainSim m_drivetrain{
      frc::DCMotor::KrakenX60(1),
      DriveConstants::kGearRatio,
      DriveConstants::Sim::kMoi,
      DriveConstants::Sim::kMass,
      units::meter_t{DriveConstants::kWheelDiameterMeters / 2.0},
      DriveConstants::kTrackWidth * DriveConstants::Sim::kScrubFactor};

  hal::SimDouble m_gyroYaw;
};
