// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <frc/DigitalInput.h>
#include <Constants.h>
#include <networktables/NetworkTableEntry.h>

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  /**
   * Picks up game pieces
  */
  IntakeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Sets the motor's power (between -1.0 and 1.0).
  void SetIntakeMotorPower(double power);

  /**
   * @return If light sensor has detected a coral
   */
  bool CoralInIntake();

  double GetDirection();

 private:

  void ConfigureIntakeSparkMax();

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Motor Controllers
  rev::spark::SparkMax m_rightIntakeSparkMax;
  rev::spark::SparkMax m_leftIntakeSparkMax;

  // Light Sensor is a digital input in the DIO port (digital input output)
  frc::DigitalInput m_lightSensor{IntakeConstants::kLightSensorID};

  // Network Table entry
  nt::NetworkTableEntry nte_coralInIntake;
};
