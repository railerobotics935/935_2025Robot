// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <frc/DigitalInput.h>
#include <Constants.h>

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  /**
   * Picks up game pieces
  */
  ClimberSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Sets the motor's power (between -1.0 and 1.0).
   */ 
  void SetMotorPower(double power);

    //  /**
    //   * @return Direction pitch motor is moving
    //   */
    //  double GetDirection();

  /**
   * @returns square of input with the same sign
   */
  double SignedSquare(double input);

 private:

  void ConfigureClimberSparkMax();

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Motor Controllers
  rev::spark::SparkMax m_climberSparkMax;
  //  rev::spark::SparkMax m_pitchSparkMax;

  // Encoders
  //rev::spark::SparkAbsoluteEncoder m_pitchAbsoluteEncoder = m_pitchSparkMax.GetAbsoluteEncoder();

  // Light Sensor is a digital input in the DIO port (digital input output)
  //  frc::DigitalInput m_lightSensor{ClimberConstants::kLightSensorID};

  //PID for the pitch
  // rev::spark::SparkClosedLoopController m_pitchPIDController = m_pitchSparkMax.GetClosedLoopController();
};
