#include "subsystems/IntakeSubsystem/IntakeSubsystem.h"

#include "rev/config/SparkMaxConfig.h"

IntakeSubsystem::IntakeSubsystem():
Intake{IntakeConstants::IntakePort, rev::spark::SparkLowLevel::MotorType::kBrushless}
{
    rev::spark::SparkMaxConfig config{};
    
    config.Inverted(false)
    .SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);

    Intake.Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                         rev::spark::SparkMax::PersistMode::kPersistParameters);
};


   

void IntakeSubsystem::On(){
    intake.Set(1.0);
};
void IntakeSubsystem::Off(){
    intake.Set(0.0);
};
