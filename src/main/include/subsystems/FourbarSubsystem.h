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
   * Creates a Elevator subsystem.
   * Currently for both indiviual elevators (two phystical subsystems)
   * but coding it as one
  */

  bool FourBarAtBase();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override; 

  /**
   * @returns True if the base elevator limit switch is pressed
  */
  bool ElevatorAtBase();

  /**
   * @returns True if the upper elevator limit switch is pressed
  */
  bool ElevatorRisen();

  /**
   * Set the elevator motor to a power
   * 
   * @param power Power to set the motor power
  */
  void SetFourBarPower(double power);

  /**
   * Set the elevator motor power invidualy
  */
  void SetIndividualElevatorPower(double power);

  /**
   * Updates NetworkTableEntries
  */
  void UpdateNTE();

 private:

  nt::NetworkTableEntry m_FourBarLimitSwitch;
  nt::NetworkTableEntry m_FourBarDistance;

  // Motor Controllers
  rev::spark::SparkMax m_leftFourBarSparkMax{FourBarConstants::kLeftBarMotorID, FourBarConstants::kMotorType};
  rev::spark::SparkMax m_rightFourBarSparkMax{FourBarConstants::kRightBarMotorID, FourBarConstants::kMotorType};

  
  // Encoders motor controllers
  frc::Encoder m_fourBarEncoder{FourBarConstants::kFourBarSensA, FourBarConstants::kFourBarSensB};  

  // // Limit switch is a digital input in the DIO port (digital input output)
  // frc::DigitalInput m_LimitSwitch{FourBarConstants::kLimitSwitchPort};

};