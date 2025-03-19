// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakePitchSubsystem.h"
#include "Constants.h"

IntakePitchSubsystem::IntakePitchSubsystem() 
: m_intakePitchSparkMax{IntakeConstants::kPitchMotorID, IntakeConstants::kMotorType} {

  ConfigureIntakePitchSparkMax();

  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("Intake");

  nte_intakePitchEncoderValue = nt_table->GetEntry("Intake/Pitch Encoder Value");

} 

  // Implementation of subsystem constructor goes here.


void IntakePitchSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.

  nte_intakePitchEncoderValue.SetDouble(m_intakePitchEncoder.GetPosition());

}

 void IntakePitchSubsystem::SetPitchPosition(double setAngle) {
  // Use the Spark MAX internal PID controller to reach the setAngle
  // - downward is counterclockwise along the robot Y-axis => positive direction for Angle
  // - check intake rotates down when applying a positive value to the SetPitchPower method,
  //   if this is not the case, invert the Pitch Spark MAX motor controller.
  // - check absolute encoder on the pitch angle to have an increasing value when rotating downward
  //   if this is not the case, add minus sign to m_pitchAbsoluteEncoder.GetPosition() references.

  // Automatically disable control control direction goes outside mechanical operating limits
  // - IntakeConstants::kMinimumAngle sets the upper pitch limit, this is the lowest angle value
  // - IntakeConstants::kMaximumAngle sets the downward pitch limit, this is the highest angle value

  // Limit Pitch going too far up
  
  if (setAngle < IntakeConstants::kMaximumAngle) {
    setAngle = IntakeConstants::kMaximumAngle;
  }

  //The angle increases as intake lowers

  // Limit Pitch going too far down
  if (setAngle > IntakeConstants::kMinimumAngle) {
    setAngle = IntakeConstants::kMinimumAngle;
  }
  

  m_intakePitchPIDController.SetReference(setAngle, rev::spark::SparkLowLevel::ControlType::kPosition);

}

void IntakePitchSubsystem::SetPitchPower(double power) {

  if ((m_intakePitchEncoder.GetPosition() < IntakeConstants::kMaximumAngle) &&
  (power < 0.0)) {
    power = 0.0;
  }

  if ((m_intakePitchEncoder.GetPosition() > IntakeConstants::kMinimumAngle) &&
  (power > 0.0)) {
    power = 0.0;
  }

 m_intakePitchSparkMax.Set(power);


}

double IntakePitchSubsystem::GetIntakeAngle() {
  return m_intakePitchEncoder.GetPosition();
}

void IntakePitchSubsystem::ConfigureIntakePitchSparkMax() {
  // Configure the Intake Spark MAX
  rev::spark::SparkMaxConfig intakePitchSparkMaxConfig{};

  intakePitchSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompensationValue)
  .SetIdleMode(IntakeConstants::kIntakeMotorIdleMode)
  .SmartCurrentLimit(IntakeConstants::kIntakeMotorCurrentLimit.value());

  intakePitchSparkMaxConfig.closedLoop
  .Pidf(IntakeConstants::kPitchP, IntakeConstants::kPitchI, IntakeConstants::kPitchD, IntakeConstants::kPitchFF)
  .OutputRange(IntakeConstants::kPitchMinOutput, IntakeConstants::kPitchMaxOutput);

  m_intakePitchSparkMax.Configure(intakePitchSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

}
