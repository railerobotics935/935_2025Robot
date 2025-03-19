// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <frc/DigitalInput.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include <Constants.h>

class IntakePitchSubsystem : public frc2::SubsystemBase {
 public:
  /**
   * Picks up game pieces
  */
  IntakePitchSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Sets the motor's power (between -1.0 and 1.0).
  void SetPitchPower(double power);

  // Sets position with PID
  void SetPitchPosition(double setAngle);

  double GetIntakeAngle();

 private:

  void ConfigureIntakePitchSparkMax();

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Motor Controllers
  rev::spark::SparkMax m_intakePitchSparkMax;

  // Encoder for Pitch 
  rev::spark::SparkAbsoluteEncoder m_intakePitchEncoder = m_intakePitchSparkMax.GetAbsoluteEncoder();

  // PID for Intake Pitch
  rev::spark::SparkClosedLoopController m_intakePitchPIDController = m_intakePitchSparkMax.GetClosedLoopController();

  // Network Table Entries
  nt::NetworkTableEntry nte_intakePitchEncoderValue;

};
