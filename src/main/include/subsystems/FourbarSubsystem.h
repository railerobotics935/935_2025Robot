// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <units/length.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/DigitalInput.h>
#include <frc/SensorUtil.h>
#include <frc/Encoder.h>

#include "Constants.h"

class FourBarSubsystem : public frc2::SubsystemBase {
 public:
  FourBarSubsystem();
  /**
   * Creates the fourbar subsystem.
   * Currently for both indiviual fourbars (two phystical subsystems)
   * but coding it as one
  */

  bool FourBarAtBase();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override; 


  /**
   * Set the fourbar motor to a power
   * 
   * @param power Power to set the motor power
  */
  void SetFourBarPower(double power);

  /**
   * Set the fourbar height
   * 
   * @param height Height to set the fourbar
   */
  void SetFourBarHeight(double height);

// Gets the height of the FourBar via encoder 
  double GetFourBarHeight ();

  /**
   * Updates NetworkTableEntries
  */
  void UpdateNTE();

 private:

  //  nt::NetworkTableEntry nte_FourBarLimitSwitch;
  nt::NetworkTableEntry nte_leftFourBarDistance;
  nt::NetworkTableEntry nte_rightFourBarDistance;

  // Motor Controllers
  rev::spark::SparkMax m_leftFourBarSparkMax{FourBarConstants::kLeftBarMotorID, FourBarConstants::kMotorType};
  rev::spark::SparkMax m_rightFourBarSparkMax{FourBarConstants::kRightBarMotorID, FourBarConstants::kMotorType};

  // PID Controller for FourBar
  rev::spark::SparkClosedLoopController m_fourBarPIDController = m_leftFourBarSparkMax.GetClosedLoopController();
  
  // Encoders motor controllers
  //  frc::Encoder m_fourBarEncoder{FourBarConstants::kFourBarSensA, FourBarConstants::kFourBarSensB};  
  rev::spark::SparkAbsoluteEncoder m_leftBarEncoder = m_leftFourBarSparkMax.GetAbsoluteEncoder();
  rev::spark::SparkAbsoluteEncoder m_rightBarEncoder = m_rightFourBarSparkMax.GetAbsoluteEncoder();


};