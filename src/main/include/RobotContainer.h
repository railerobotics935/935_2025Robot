// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/PowerDistribution.h>

#include "Constants.h"
#include "utils/SwerveUtils.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ClimberSubsystem.h"
#include "subsystems/FourBarSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/IntakePitchSubsystem.h"

#include "commands/drive/DriveWithController.h"
#include "commands/drive/DriveWithController.h"
#include "commands/climber/SimpleClimb.h"
#include "commands/climber/SimpleDescend.h"
#include "commands/climber/StopClimber.h"
#include "commands/fourbar/LowerFourBar.h"
#include "commands/fourbar/RaiseFourBar.h"
#include "commands/fourbar/StopFourBar.h"
#include "commands/intake/IntakeWIthTriggers.h"
#include "commands/intake/RaiseIntakePitch.h"
#include "commands/intake/LowerIntakePitch.h"
#include "commands/intake/StopIntakePitch.h"
#include "commands/autocommands/SimpleIntake.h"
#include "commands/autocommands/SimpleOuttake.h"
#include "commands/autocommands/FourBarToL4.h"
#include "commands/autocommands/FourBarToIntake.h"
#include "commands/autocommands/WristToIntake.h"
#include "commands/autocommands/WristToL4.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  /**
   * @return The command for autonomous
  */
  frc2::CommandPtr GetAutonomousCommand();

 private:
  /**
   * Convigures button bindings for commands
  */
  void ConfigureButtonBindings();

  // The driver and operator controllers
  frc::XboxController m_driveController{OIConstants::kDriverControllerPort};
  frc::XboxController m_operatorController{OIConstants::kOperatorControllerPort};

  frc::PowerDistribution m_revPDH{1, frc::PowerDistribution::ModuleType::kRev};
  // Variables 
  bool isFieldRelative = true;

  // The robot's subsystems
  DriveSubsystem m_drive;
  ClimberSubsystem m_climber;
  FourBarSubsystem m_fourBar;
  IntakeSubsystem m_intake;
  IntakePitchSubsystem m_intakePitch;


  // Sendable chooser for auto
  frc::SendableChooser<std::string> m_autoChooser;

  DriveWithController m_driveWithController{&m_drive, &m_driveController};
  StopFourBar m_stopFourBar{&m_fourBar};
  RaiseFourBar m_raiseFourBar{&m_fourBar};
  LowerFourBar m_lowerFourBar{&m_fourBar};
  SimpleClimb m_simpleClimb{&m_climber};
  SimpleDescend m_simpleDescend{&m_climber};
  StopClimber m_stopClimber{&m_climber};
  IntakeWithTriggers m_intakeWithTriggers{&m_intake, &m_operatorController};
  RaiseIntakePitch m_raiseIntakePitch{&m_intakePitch};
  LowerIntakePitch m_lowerIntakePitch{&m_intakePitch};
  StopIntakePitch m_stopIntakePitch{&m_intakePitch};
  SimpleIntake m_simpleIntake{&m_intake};
  SimpleOuttake m_simpleOuttake{&m_intake};
  FourBarToL4 m_fourBarToL4{&m_fourBar};
  FourBarToIntake m_fourBarToIntake{&m_fourBar};
  SetWristToIntake m_setWristToIntake{&m_intakePitch};
  WristToL4 m_wristToL4{&m_intakePitch};

};
