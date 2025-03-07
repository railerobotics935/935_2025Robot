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

  m_leftFourBarSparkMax.Configure(leftBarSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);  

  rev::spark::SparkMaxConfig rightBarSparkMaxConfig{};

  rightBarSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompensationValue)
  .SetIdleMode(kRightBarMotorIdleMode)
  .SmartCurrentLimit(kRightBarMotorCurrentLimit.value());

  rightBarSparkMaxConfig.encoder
  .PositionConversionFactor(kRightBarEncoderPositionFactor)
  .VelocityConversionFactor(kRightBarEncoderVelocityFactor);

  m_rightFourBarSparkMax.Configure(rightBarSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);  


 // m_elevatorSparkMax.BurnFlash();

  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  
  auto nt_table = nt_inst.GetTable("Elevator");

  m_FourBarLimitSwitch = nt_table->GetEntry("Elevator/Limit Switch");
  m_FourBarDistance = nt_table->GetEntry("Elevator/Distance Extended");

    // Set the distance per pulse if needed
  m_fourBarEncoder.SetDistancePerPulse(0.02 / 360.0); // Example for a 360 PPR encoder
  
}

bool FourBarSubsystem::FourBarAtBase() {
//  return !m_LimitSwitch.Get();
  if(m_fourBarEncoder.Get() == 0) { // encoder value will need to be changed
    return true;
  }
  else {
    return false;
  }
}

void FourBarSubsystem::Periodic() {
  UpdateNTE();

//  if (FourBarAtBase())
//   m_fourBarEncoder.Reset(); I don't think this works for our encoders but not sure
}

void FourBarSubsystem::UpdateNTE() {
  m_FourBarLimitSwitch.SetBoolean(FourBarAtBase());
  m_FourBarDistance.SetDouble(m_fourBarEncoder.GetDistance());
}

void FourBarSubsystem::SetFourBarPower(double power) {
  /*if (power < 0.0 && m_elevatorEncoder.GetDistance() < -6.2) {
   m_elevatorSparkMax.Set(0.0);
  }
  else {
    if (ElevatorAtBase() && power > 0.0)
     m_elevatorSparkMax.Set(0.0);

    else {
     */
    m_leftFourBarSparkMax.Set(power);
    m_rightFourBarSparkMax.Set(power);
    }
  //}}
  

