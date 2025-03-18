// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/SparkMax.h>
#include <frc/DigitalInput.h>
#include <Constants.h>

class IntakePitchSubsystem : public frc2::SubsystemBase {
 public:

 IntakePitchSubsystem();

 void Periodic() override;

 // Sets the angle of the  intake
  void SetIntakeAngle(double angle);

  // Gets the angle of the  intake
  double GetIntakeAngle();

  void SetPitchPower(double power);

private:

  void ConfigureSparkMax();

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Motor Controllers
  rev::spark::SparkMax m_pitchSparkMax;

  // Encoders
  rev::spark::SparkAbsoluteEncoder m_pitchAbsoluteEncoder = m_pitchSparkMax.GetAbsoluteEncoder();

  //PID for the pitch
  rev::spark::SparkClosedLoopController m_pitchPIDController = m_pitchSparkMax.GetClosedLoopController();

  nt::NetworkTableEntry nte_pitchEncoderValue;
};

