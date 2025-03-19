// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"
#include <networktables/NetworkTableInstance.h>

IntakeSubsystem::IntakeSubsystem() 
: m_rightIntakeSparkMax{IntakeConstants::kRightIntakeMotorID, IntakeConstants::kMotorType},
  m_leftIntakeSparkMax{IntakeConstants::kLeftIntakeMotorID, IntakeConstants::kMotorType} {

    ConfigureIntakeSparkMax();

  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("Intake");

  nte_coralInIntake = nt_table->GetEntry("Intake/Coral in Intake");

} 

  // Implementation of subsystem constructor goes here.


void IntakeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  nte_coralInIntake.SetBoolean(CoralInIntake());
}

void IntakeSubsystem::SetIntakeMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_leftIntakeSparkMax.Set(power);
}

double IntakeSubsystem::GetDirection() {
  return m_leftIntakeSparkMax.Get();
}


bool IntakeSubsystem::CoralInIntake() {
  return m_lightSensor.Get();
}

void IntakeSubsystem::ConfigureIntakeSparkMax() {
  // Configure the Intake Spark MAX
  rev::spark::SparkMaxConfig rightIntakeSparkMaxConfig{};
  rev::spark::SparkMaxConfig leftIntakeSparkMaxConfig{};


  rightIntakeSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompensationValue)
  .SetIdleMode(IntakeConstants::kIntakeMotorIdleMode)
  .SmartCurrentLimit(IntakeConstants::kIntakeMotorCurrentLimit.value())
  .Follow(IntakeConstants::kLeftIntakeMotorID, true);

  leftIntakeSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompensationValue)
  .SetIdleMode(IntakeConstants::kIntakeMotorIdleMode)
  .SmartCurrentLimit(IntakeConstants::kIntakeMotorCurrentLimit.value());

  m_rightIntakeSparkMax.Configure(rightIntakeSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
  m_leftIntakeSparkMax.Configure(leftIntakeSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

}
