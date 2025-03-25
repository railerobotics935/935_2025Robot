// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSubsystem.h"
#include "Constants.h"

using namespace ClimberConstants;

ClimberSubsystem::ClimberSubsystem() : m_climberSparkMax{kClimberMotorID, kMotorType}  {
  // Implementation of subsystem constructor goes here.
 
  ConfigureClimberSparkMax();

}

void ClimberSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void ClimberSubsystem::SetMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_climberSparkMax.Set(power);
}

// double ClimberSubsystem::SignedSquare(double input) {
//   if (input > 0) {
//     return std::pow(input, 2);
//   }
//   else {
//     return -std::pow(input, 2);
//   }
// }

void ClimberSubsystem::ConfigureClimberSparkMax() {
  // Configure the Climber Spark MAX
  rev::spark::SparkMaxConfig climberSparkMaxConfig{};

  climberSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompensationValue)
  .SetIdleMode(kClimberMotorIdleMode)
  .SmartCurrentLimit(kClimberMotorCurrentLimit.value());

  //climberSparkMaxConfig.encoder
  //.PositionConversionFactor(kClimberPositionFactor)
  //.VelocityConversionFactor(kClimberVelocityFactor);

  m_climberSparkMax.Configure(climberSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

}
