// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"

IntakeSubsystem::IntakeSubsystem() 
: m_rightIntakeSparkMax{IntakeConstants::kRightIntakeMotorID, IntakeConstants::kMotorType},
  m_leftIntakeSparkMax{IntakeConstants::kLeftIntakeMotorID, IntakeConstants::kMotorType},
  m_pitchSparkMax{IntakeConstants::kPitchMotorID, IntakeConstants::kMotorType} {
    #ifdef BURNINTAKESPARKMAX
      ConfigureIntakeSparkMax();
    #endif
} 

  // Implementation of subsystem constructor goes here.


void IntakeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void IntakeSubsystem::SetIntakeMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_leftIntakeSparkMax.Set(power);
}

 void IntakeSubsystem::SetPitchPosition(double setAngle) {
  // Use the Spark MAX internal PID controller to reach the setAngle
  // - downward is counterclockwise along the robot Y-axis => positive direction for Angle
  // - check intake rotates down when applying a positive value to the SetPitchPower method,
  //   if this is not the case, invert the Pitch Spark MAX motor controller.
  // - check absolute encoder on the pitch angle to have an increasing value when rotating downward
  //   if this is not the case, add minus sign to m_pitchAbsoluteEncoder.GetPosition() references.
  m_pitchPIDController.SetReference(setAngle.value(), rev::spark::SparkLowLevel::ControlType::kPosition);

  // Automatically disable control control direction goes outside mechanical operating limits
  // - IntakeConstants::kMinimumAngle sets the upper pitch limit, this is the lowest angle value
  // - IntakeConstants::kMaximumAngle sets the downward pitch limit, this is the highest angle value

  // Limit Pitch going too far up
  if (setAngle < IntakeConstants::kMinimumAngle) {
   setAngle = IntakeConstants::kMinimumAngle;
  }

  // Limit Pitch going too far down
  if ((m_pitchAbsoluteEncoder.GetPosition() > IntakeConstants::kMaximumAngle) &&
      (m_pitchSparkMax.Get() > 0.0)) {
    m_pitchSparkMax.Set(0.0);
  }
}

/*
void IntakeSubsystem::SetPitchPower(double power) {
  m_pitchSparkMax.Set(power);
}
*/

double IntakeSubsystem::SignedSquare(double input) {
  if (input > 0) {
    return std::pow(input, 2);
  }
  else {
    return -std::pow(input, 2);
  }
}

/*bool IntakeSubsystem::CoralInIntake() {
  return m_lightSensor.Get();
}
*/
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

  //intakeSparkMaxConfig.encoder
  //.PositionConversionFactor(kIntakePositionFactor)
  //.VelocityConversionFactor(kIntakeVelocityFactor);

  m_rightIntakeSparkMax.Configure(rightIntakeSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
  m_leftIntakeSparkMax.Configure(rightIntakeSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

  // Configure the Pitch Spark MAX
  rev::spark::SparkMaxConfig pitchSparkMaxConfig{};

  pitchSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompensationValue)
  .SetIdleMode(IntakeConstants::kIntakeMotorIdleMode)
  .SmartCurrentLimit(IntakeConstants::kIntakeMotorCurrentLimit.value());

  pitchSparkMaxConfig.closedLoop
  .Pidf(IntakeConstants::kPitchP, IntakeConstants::kPitchI, IntakeConstants::kPitchD, IntakeConstants::kPitchFF)
  .OutputRange(IntakeConstants::kPitchMinOutput, IntakeConstants::kPitchMaxOutput)
  .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder);

  m_pitchSparkMax.Configure(pitchSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}
/*
double IntakeSubsystem::GetDirection() {
  return m_pitchSparkMax.Get();
}
*/