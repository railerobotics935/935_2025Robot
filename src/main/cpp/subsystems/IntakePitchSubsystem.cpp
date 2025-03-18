// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/shuffleboard/Shuffleboard.h>
#include "subsystems/IntakePitchSubsystem.h"
#include "Constants.h"

IntakePitchSubsystem::IntakePitchSubsystem() 

: m_pitchSparkMax{IntakeConstants::kPitchMotorID, IntakeConstants::kMotorType} {
  // Implementation of subsystem constructor goes here.

  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("Intake");

  nte_pitchEncoderValue = nt_table->GetEntry("/Pitch Encoder Value");

   #ifdef BURNPITCHSPARKMAX
  rev::spark::SparkMaxConfig pitchSparkMaxConfig{};

  pitchSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
  .SetIdleMode(IntakeConstants::kPitchMotorIdleMode)
  .SmartCurrentLimit(IntakeConstants::kIntakeMotorCurrentLimit.value());

  pitchSparkMaxConfig.closedLoop
  .Pidf(IntakeConstants::kPitchP, IntakeConstants::kPitchI, IntakeConstants::kPitchD, IntakeConstants::kPitchFF)
  .OutputRange(IntakeConstants::kPitchMinOutput, IntakeConstants::kPitchMaxOutput)
  .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder);
  
   m_pitchSparkMax.Configure(pitchSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

  std::cout << "Flash Burned on pitch subsystem\r\n";
  #else
  std::cout << "Flash was not burned on pitch subsystem\r\n";
  #endif
}

void IntakePitchSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  nte_pitchEncoderValue.SetDouble(m_pitchAbsoluteEncoder.GetPosition());
}

double IntakePitchSubsystem::GetIntakeAngle() {
  // Gets the angle of the  intake from encoder 
  return m_pitchAbsoluteEncoder.GetPosition();
}

void IntakePitchSubsystem::SetIntakeAngle(double angle) {
  // Use the Spark MAX internal PID controller to reach the setAngle
  // - downward is counterclockwise along the robot Y-axis => positive direction for Angle
  // - check intake rotates down when applying a positive value to the SetPitchPower method,
  //   if this is not the case, invert the Pitch Spark MAX motor controller.
  // - check absolute encoder on the pitch angle to have an increasing value when rotating downward
  //   if this is not the case, add minus sign to m_pitchAbsoluteEncoder.GetPosition() references.
  // Automatically disable control control direction goes outside mechanical operating limits
  // - IntakeConstants::kMinimumAngle sets the upper pitch limit, this is the lowest angle value
  // - IntakeConstants::kMaximumAngle sets the downward pitch limit, this is the highest angle value

  // Limit Pitch going too far down
  if (angle < IntakeConstants::kMinimumAngle) {
    angle = IntakeConstants::kMinimumAngle;
  }

  // Limit Pitch going too far up
  if (angle > IntakeConstants::kMaximumAngle) {
    angle = IntakeConstants::kMaximumAngle;
  }

   m_pitchPIDController.SetReference(angle, rev::spark::SparkLowLevel::ControlType::kPosition);
}

void IntakePitchSubsystem::SetPitchPower(double power) {
  m_pitchSparkMax.Set(power);
}
