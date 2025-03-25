// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <utility>

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include "pathplanner/lib/commands/PathPlannerAuto.h"
#include "pathplanner/lib/auto/NamedCommands.h"

#include "subsystems/DriveSubsystem.h"
#include "commands/autocommands/SimpleAuto.h"
#include "Constants.h"

using namespace DriveConstants;
using namespace pathplanner;

/**
 * Idea:
 * 
 * m_drive.SetDefaultCommand(std::move(m_driveCommand));
 *   
 * it says it works, but it hasen't been tested yet. I don't know how different it
 * is, if it is better or not
*/

RobotContainer::RobotContainer() {
  m_revPDH.SetSwitchableChannel(true); //-------------------------------------------------------------------------------------

  
  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(std::move(m_driveWithController));
  m_climber.SetDefaultCommand(std::move(m_stopClimber));
  m_fourBar.SetDefaultCommand(std::move(m_stopFourBar));
  m_intake.SetDefaultCommand(std::move(m_intakeWithTriggers));
  m_intakePitch.SetDefaultCommand(std::move(m_stopIntakePitch));

  NamedCommands::registerCommand("Simple Intake", std::move(m_simpleIntake).ToPtr());
  NamedCommands::registerCommand("Simple Outtake", std::move(m_simpleOuttake).ToPtr());
  NamedCommands::registerCommand("Four Bar To L4", std::move(m_fourBarToL4).ToPtr());
  NamedCommands::registerCommand("Four Bar to Intake", std::move(m_fourBarToIntake).ToPtr());
  NamedCommands::registerCommand("Set Wrist To L4", std::move(m_wristToL4).ToPtr());
  NamedCommands::registerCommand("Set Wrist to Intake", std::move(m_setWristToIntake).ToPtr());
  NamedCommands::registerCommand("Reset Home", std::move(m_resetHome).ToPtr());
  
  frc::Shuffleboard::GetTab("Autonomous").Add(m_autoChooser);

  m_autoChooser.SetDefaultOption("F Leave", m_fLeave);
  m_autoChooser.AddOption("C Leave", m_cLeave);
  m_autoChooser.AddOption("M Leave", m_mLeave);
  m_autoChooser.AddOption("C5 Escape", m_c5escape); 
  m_autoChooser.AddOption("C 6R&5L Escape", m_c6r5l);
}

void RobotContainer::ConfigureButtonBindings() {

  // Create new button bindings
  frc2::JoystickButton resetButton(&m_driveController, ControllerConstants::kResetGyroButtonIndex); 
  frc2::JoystickButton robotRelativeButton(&m_driveController, ControllerConstants::kRobotRelativeButtonIndex);
  frc2::JoystickButton fieldRelativeButton(&m_driveController, ControllerConstants::kFieldRelativeButtonIndex); 
  frc2::JoystickButton raiseClimberButton(&m_driveController, ControllerConstants::kRaiseClimberButtonIndex);
  frc2::JoystickButton lowerClimberButton(&m_driveController, ControllerConstants::kLowerClimberButtonIndex);
  frc2::JoystickButton raiseFourBarButton(&m_operatorController, ControllerConstants::kRaiseFourBarButtonIndex);
  frc2::JoystickButton lowerFourBarButton(&m_operatorController, ControllerConstants::kLowerFourBarButtonIndex);
  frc2::JoystickButton raiseIntakePitchButton(&m_operatorController, ControllerConstants::kRaiseIntakePitchIndex);
  frc2::JoystickButton lowerIntakePitchButton(&m_operatorController, ControllerConstants::kLowerIntakePitchIndex);

  // Bind commands to button triggers
  resetButton.OnTrue(frc2::cmd::RunOnce([&] {m_drive.ZeroHeading();}, {}));
  robotRelativeButton.OnTrue(frc2::cmd::RunOnce([&] {m_drive.SetRobotRelative();}, {}));
  fieldRelativeButton.OnTrue(frc2::cmd::RunOnce([&] {m_drive.SetFieldRelative();}, {}));
  lowerClimberButton.WhileTrue(SimpleDescend{&m_climber}.ToPtr());
  raiseClimberButton.WhileTrue(SimpleClimb{&m_climber}.ToPtr());
  lowerFourBarButton.WhileTrue(LowerFourBar{&m_fourBar}.ToPtr());
  raiseFourBarButton.WhileTrue(RaiseFourBar{&m_fourBar}.ToPtr());
  raiseIntakePitchButton.WhileTrue(RaiseIntakePitch{&m_intakePitch}.ToPtr());
  lowerIntakePitchButton.WhileTrue(LowerIntakePitch{&m_intakePitch}.ToPtr());

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // Builds and returns auto commands from pathplanner
  return (SimpleAuto{&m_drive, &m_fourBar, &m_intakePitch}.ToPtr());
}
