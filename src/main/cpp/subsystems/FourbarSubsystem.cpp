// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/FourBarSubsystem.h"
#include "Constants.h"
#include <networktables/NetworkTableInstance.h>

using namespace FourBarConstants;

FourBarSubsystem::FourBarSubsystem() {

  // Restore defaults
rev::spark::SparkMaxConfig leftBarSparkMaxConfig{};
// rev::spark::SparkAbsoluteEncoder forbarAbsoluteEncoder{};

  leftBarSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompensationValue)
  .SetIdleMode(kLeftBarMotorIdleMode)
  .SmartCurrentLimit(kLeftBarMotorCurrentLimit.value());

  leftBarSparkMaxConfig.encoder
  .PositionConversionFactor(kLeftBarEncoderPositionFactor)
  .VelocityConversionFactor(kLeftBarEncoderVelocityFactor);

  leftBarSparkMaxConfig.absoluteEncoder
  .Inverted(true)
  .ZeroOffset(0.670)
  .PositionConversionFactor(kLeftBarEncoderPositionFactor)
  .VelocityConversionFactor(kLeftBarEncoderVelocityFactor);

  leftBarSparkMaxConfig.closedLoop
  .Pidf(kFourBarP, kFourBarI, kFourBarD, kFourBarFF)
  .OutputRange(kMinimumOutput, kMaximumOutput)
  .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder);
  
  m_leftFourBarSparkMax.Configure(leftBarSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);  

  rev::spark::SparkMaxConfig rightBarSparkMaxConfig{};

  #ifdef FOURBAR_LR_INDEPENDENT
  rightBarSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompensationValue)
  .SetIdleMode(kRightBarMotorIdleMode)
  .SmartCurrentLimit(kRightBarMotorCurrentLimit.value());
  #else
  rightBarSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompensationValue)
  .SetIdleMode(kRightBarMotorIdleMode)
  .SmartCurrentLimit(kRightBarMotorCurrentLimit.value())
  .Follow(kLeftBarMotorID, false); // This boolean decides if the right is inverted from the left, change it if it's wrong
  #endif

  rightBarSparkMaxConfig.encoder
  .PositionConversionFactor(kRightBarEncoderPositionFactor)
  .VelocityConversionFactor(kRightBarEncoderVelocityFactor);

  rightBarSparkMaxConfig.absoluteEncoder
  .Inverted(false)
  .ZeroOffset(0.026)
  .PositionConversionFactor(kRightBarEncoderPositionFactor)
  .VelocityConversionFactor(kRightBarEncoderVelocityFactor);

  rightBarSparkMaxConfig.closedLoop
  .Pidf(kFourBarP, kFourBarI, kFourBarD, kFourBarFF)
  .OutputRange(kMinimumOutput, kMaximumOutput)
  .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder);

  m_rightFourBarSparkMax.Configure(rightBarSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);


  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  
  auto nt_table = nt_inst.GetTable("FourBar");

  //m_FourBarLimitSwitch = nt_table->GetEntry("Elevator/Limit Switch");
  nte_leftFourBarDistance = nt_table->GetEntry("FourBar/Left Distance Extended");
  nte_rightFourBarDistance = nt_table->GetEntry("FourBar/Right Distance Extended");

    // Set the distance per pulse if needed
  //m_leftBarEncoder.SetDistancePerPulse(0.02 / 360.0); // Example for a 360 PPR encoder
  
}

bool FourBarSubsystem::FourBarAtBase() {
//  return !m_LimitSwitch.Get();
  if(m_leftBarEncoder.GetPosition() >= 0.5 && m_rightBarEncoder.GetPosition() <= 0.0) { // encoder value will need to be changed
    return true;
  }
  else {
    return false;
  }
}

void FourBarSubsystem::Periodic() {
  UpdateNTE();

//  if (FourBarAtBase())
//   m_leftBarEncoder.Reset(); //I don't think this works for our encoders but not sure

}

void FourBarSubsystem::UpdateNTE() {
//  m_FourBarLimitSwitch.SetBoolean(FourBarAtBase());
  nte_leftFourBarDistance.SetDouble(m_leftBarEncoder.GetPosition());
  nte_rightFourBarDistance.SetDouble(m_rightBarEncoder.GetPosition());
}

void FourBarSubsystem::SetFourBarPower(double power) {
  
    //m_leftFourBarSparkMax.Set(power);
  #ifdef FOURBAR_LR_INDEPENDENT
  m_leftFourBarSparkMax.SetVoltage(filter.Calculate((units::volt_t)power * 11.0));
  m_rightFourBarSparkMax.SetVoltage(filter.Calculate((units::volt_t)power * 11.0));
  #else
  m_leftFourBarSparkMax.SetVoltage(filter.Calculate((units::volt_t)power * 11.0));
  #endif

}

  
void FourBarSubsystem::SetFourBarHeight(double height) {
  //Prevents FourBar from going too high
  /*if (height < kMaximumHeight) {
    height = kMaximumHeight;
  }
  // Prevents FourBar from going too low
  if (height > kMinimumHeight) {
    height = kMinimumHeight;
  }
*/  
  #ifdef FOURBAR_LR_INDEPENDENT
  m_leftFourBarPIDController.SetReference(height, rev::spark::SparkLowLevel::ControlType::kPosition);
  m_rightFourBarPIDController.SetReference(height, rev::spark::SparkLowLevel::ControlType::kPosition);
//  m_rightFourBarPIDController.SetReference(0.664 - height, rev::spark::SparkLowLevel::ControlType::kPosition);

  #else
  m_leftFourBarPIDController.SetReference(height, rev::spark::SparkLowLevel::ControlType::kPosition);

  #endif

}

  double FourBarSubsystem::GetFourBarHeight() {
  // returns the position of the leader bar encoder (should apply to follower too)
  return m_leftBarEncoder.GetPosition();
}

