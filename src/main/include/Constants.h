// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/current.h>
#include <frc/geometry/Pose3d.h>
#include <frc/apriltag/AprilTagFields.h>
#include <rev/SparkMax.h>
#include <iostream>
#include <rev/config/SparkMaxConfig.h>



// Turn this off when there is no new constants need to be burned onto motorcontrollers
#define BURNMODULESPARKMAX
#define USEXBOXCONTROLLER
#define PRINTDEBUG
//#define DEBUGPOSEESTIMATION
//#define
#define FOURBAR_LR_INDEPENDENT

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */


namespace RobotConstants {

constexpr double kVoltageCompensationValue = 11.0;

const units::meter_t kWheelBase =
    0.6223_m;  // Distance between centers of front and back wheels on robot
const units::meter_t kWheelWidth =
    0.6223_m; // Distance between centers of left and right wheels on robot

}

namespace DriveConstants {
// Driving Parameters - Note that these are not the maximum capable speeds of
// the robot, rather the allowed maximum speeds
constexpr units::meters_per_second_t kMaxSpeed = 4.3_mps;
constexpr units::radians_per_second_t kMaxAngularSpeed{2.0 * std::numbers::pi};

constexpr double kDirectionSlewRate = 6.0;   // radians per second
constexpr double kMagnitudeSlewRate = 7.0;   // percent per second (1 = 100%)
constexpr double kRotationalSlewRate = 8.0;  // percent per second (1 = 100%)

// CAN Sparkmax id numbers
constexpr int kFrontLeftDriveMotorPort = 28;
constexpr int kFrontRightDriveMotorPort = 20;
constexpr int kBackLeftDriveMotorPort = 10;
constexpr int kBackRightDriveMotorPort = 18;

constexpr int kFrontLeftTurningMotorPort = 29;
constexpr int kFrontRightTurningMotorPort = 21;
constexpr int kBackLeftTurningMotorPort = 11;
constexpr int kBackRightTurningMotorPort = 19;

// PID Controller for the auto rotation of the robot
constexpr double kRotationP = 2.5;
constexpr double kRotationI = 0.002;
constexpr double kRotationD = 0.2;

// Anolog input ports on roborio
constexpr int kFrontLeftTurningEncoderPort = kFrontLeftTurningMotorPort;
constexpr int kFrontRightTurningEncoderPort = kFrontRightTurningMotorPort;
constexpr int kBackLeftTurningEncoderPort = kBackLeftTurningMotorPort;
constexpr int kBackRightTurningEncoderPort = kBackRightTurningMotorPort;

// Offsets in radians for the encoders. the first number to to make zero forward, after that we
// subtract an additional pi to make the full range -pi to pi instead of 0 to 2pi
//constexpr double kFrontLeftDriveEncoderOffset = (1.9249 - (std::numbers::pi / 2) + (std::numbers::pi / 3)) + 0.033;
//constexpr double kFrontRightDriveEncoderOffset = (3.2676) - (std::numbers::pi / 3) - 0.062; 
//constexpr double kBackLeftDriveEncoderOffset =  (2.0477) - (2.0 * std::numbers::pi / 3) + std::numbers::pi + 0.050; //(0.6988 + (std::numbers::pi / 2)); 
//constexpr double kBackRightDriveEncoderOffset = (3.8439 + (std::numbers::pi / 2)) - 0.019; //(2.0472 + (std::numbers::pi)); 
constexpr double kFrontLeftTurnEncoderOffset = (std::numbers::pi / 2); //-(std::numbers::pi / 2); //2.789 - (std::numbers::pi / 2) - std::numbers::pi;
constexpr double kFrontRightTurnEncoderOffset = (std::numbers::pi); //4.996 - std::numbers::pi; 
constexpr double kBackLeftTurnEncoderOffset = 0; //5.756; 
constexpr double kBackRightTurnEncoderOffset = -((std::numbers::pi) / 2); //4.407 + (std::numbers::pi / 2) - std::numbers::pi;

constexpr auto kDriveBaseRadius = 0.46_m;

}  // namespace DriveConstants

namespace ModuleConstants {
// Through-hole Encoder on Spark MAX frequency-pwm input
// Invert the turning encoder, since the output shaft rotates in the opposite
// direction of the steering motor in the MAXSwerve Module.
constexpr bool kTurningEncoderInverted = true;

// Calculations required for driving motor conversion factors and feed forward
constexpr double kDrivingMotorFreeSpeedRps =
    5676.0 / 60;  // NEO free speed is 5676 RPM
constexpr units::meter_t kWheelDiameter = 0.0953_m;
constexpr units::meter_t kWheelCircumference =
    kWheelDiameter * std::numbers::pi;

// 6.75:1 Gear Ratio for Driving Motors
constexpr double kDrivingMotorReduction = 5.07;
constexpr double kDriveWheelFreeSpeedRps =
    (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) /
    kDrivingMotorReduction;

constexpr double kDrivingEncoderPositionFactor =
    (kWheelDiameter.value() * std::numbers::pi) /
    kDrivingMotorReduction;  // meters
constexpr double kDrivingEncoderVelocityFactor =
    ((kWheelDiameter.value() * std::numbers::pi) / kDrivingMotorReduction) /
    60.0;  // meters per second

constexpr double kTurningEncoderPositionFactor =
    (2 * std::numbers::pi);  // radians
constexpr double kTurningEncoderVelocityFactor =
    (2 * std::numbers::pi) / 60.0;  // radians per second

constexpr units::radian_t kTurningEncoderPositionPIDMinInput = 0_rad;
constexpr units::radian_t kTurningEncoderPositionPIDMaxInput =
    units::radian_t{kTurningEncoderPositionFactor};

constexpr double kDrivingP = 0.0;
constexpr double kDrivingI = 0.0;
constexpr double kDrivingD = 0.0;
constexpr double kDrivingFF = (1 / kDriveWheelFreeSpeedRps);
constexpr double kDrivingMinOutput = -1;
constexpr double kDrivingMaxOutput = 1;

constexpr double kTurningP = 1.5;
constexpr double kTurningI = 0.0;
constexpr double kTurningD = 0.0; //was originally 0.15
constexpr double kTurningFF = 0;
constexpr double kTurningMinOutput = -1;
constexpr double kTurningMaxOutput = 1;

constexpr rev::spark::SparkMaxConfig::IdleMode kDrivingMotorIdleMode =  rev::spark::SparkMaxConfig::IdleMode::kBrake;
constexpr rev::spark::SparkMaxConfig::IdleMode kTurningMotorIdleMode = rev::spark::SparkMaxConfig::IdleMode::kBrake;

constexpr units::ampere_t kDrivingMotorCurrentLimit = 40_A;
constexpr units::ampere_t kTurningMotorCurrentLimit = 40_A;

constexpr auto kModuleMaxAngularVelocity =  std::numbers::pi * 9_rad_per_s;  // radians per second ?????????
constexpr auto kModuleMaxAngularAcceleration = std::numbers::pi * 20_rad_per_s / 1_s;  // radians per second^2
constexpr auto kModuleMaxLinearVelocity = 4.65_mps;
}  // namespace ModuleConstants


namespace AutoConstants {

// Only Constants here are the PID constants. Look in path planner for max veleocty/acceleration constants
// PID Constants for the tranlation (X and Y movement) of the robot during auto
constexpr double kPTanslationController = 4.0; // 6.0
constexpr double kITanslationController = 0.0; // 1.7
constexpr double kDTanslationController = 0.0; // 0.0

// PID Constants for the rotation, or Yaw of the robot
constexpr double kPRotationController = 5.0; // 5.0
constexpr double kIRotationController = 0.0; // 0.0
constexpr double kDRotationController = 0.0; // 0.0

}  // namespace AutoConstants

namespace ControllerConstants {

// Controller Constants for X Box Controllers
/**
 * BUTTONS
 * A button - 1
 * B button - 2
 * X button - 3
 * Y button - 4
 * Left Bumper - 5
 * Right Bumper - 6
 * Center Left Button - 7
 * Center Right Button - 8
 * Left Joystick Button - 9
 * Right Joystick Button - 10
 * 
 * AXES
 * Left x-axis - 0, input right creates a positive output
 * Left y-axis - 1, input down creates a positive output
 * Left Trigger - 2, input in creates a positive output
 * Right Trigger - 3, input in creates a positive output
 * Right x-axis - 4, input right creates a positive output
 * Right y-axis - 5, input down creates a positive output
*/

// Axis indexes
constexpr int kDriveLeftYIndex = 1; // An input UP creates a NEGATIVE output
constexpr int kDriveLeftXIndex = 0; // An input RIGHT creates a NEGATIVE output
constexpr int kDriveRightYIndex = 5; // An input UP creates a NEGATIVE output
constexpr int kDriveRightXIndex = 4; // An input RIGHT creates a NEGATIVE output

#ifdef USEXBOXCONTROLLER
constexpr int kOperatorLeftYIndex = 1; // An input UP creates a NEGATIVE output
constexpr int kOperatorRightYIndex = 5; // An input UP creates a NEGATIVE output
constexpr int kOuttakeTriggerIndex = 2;
constexpr int kIntakeTriggerIndex = 3;
#else
constexpr int kOperatorLeftYIndex = 1; // An input UP creates a NEGATIVE output
constexpr int kOperatorRightYIndex = 3; // An input UP creates a NEGATIVE output

#endif
// Drive Controller
constexpr int kFieldRelativeButtonIndex = 7; // CL
constexpr int kRobotRelativeButtonIndex = 8; // CR
constexpr int kResetGyroButtonIndex = 2; // B
constexpr int kRaiseClimberButtonIndex = 6;
constexpr int kLowerClimberButtonIndex = 5;

// Operator Controller
constexpr int kLowerFourBarButtonIndex = 5;
constexpr int kRaiseFourBarButtonIndex = 6;
constexpr int kRaiseIntakePitchIndex = 4; // Y
constexpr int kLowerIntakePitchIndex = 3; // X

} // namespace ControllerConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr int kOperatorControllerPort = 1;
}  // namespace OIConstants

namespace CameraConstants {

// Min and Max standard deviations for the apriltag detetion 
constexpr double kMinStandardDeviation = 0.2;
constexpr double kMaxStandardDeviation = 3.0;

// Max speed allowed for adding vidion measurments to the robot pose esitmator
constexpr double kMaxEstimationSpeed = 0.25; // mps

/**
 * @param distance The raw distance from the apriltag
 * 
 * @return The standard deviation value for the distance
*/
double GetStandardDeviationFromDistance(double distance);

// Pose3d/transformation2d of the camera relative to the robot
// X if forward, Y is Left, Z is up 
namespace FrontCamera {
    const frc::Translation3d kTranlation3d{(units::meter_t)0.250, (units::meter_t)-0.185, (units::meter_t)0.2286};
    const frc::Rotation3d kRotation3d{(units::radian_t)0.0, (units::radian_t)std::numbers::pi / 12, (units::radian_t)0.0};
    const frc::Pose3d kPose3d{kTranlation3d, kRotation3d};
} // namespace FrontCamera

namespace BackLeftCamera {
    const frc::Translation3d kTranlation3d{(units::meter_t)-0.250, (units::meter_t)0.4125, (units::meter_t)0.2286};
    const frc::Rotation3d kRotation3d{(units::radian_t)std::numbers::pi * -0.1116883853, (units::radian_t)std::numbers::pi * 0.1116883853, (units::radian_t)std::numbers::pi * 1.25};
    const frc::Pose3d kPose3d{kTranlation3d, kRotation3d};
} // namespace BackLeftCamera

namespace BackRightCamera {
    const frc::Translation3d kTranlation3d{(units::meter_t)-0.250, (units::meter_t)-0.4125, (units::meter_t)0.2286};
    const frc::Rotation3d kRotation3d{(units::radian_t)std::numbers::pi * 0.1116883853, (units::radian_t)std::numbers::pi * 0.1116883853, (units::radian_t)std::numbers::pi * 0.75};
    const frc::Pose3d kPose3d{kTranlation3d, kRotation3d};
} // namespace BackRightCamera

} // namespace CameraConstants

namespace IntakeConstants {
// Intake motor 
constexpr int kRightIntakeMotorID = 23;
constexpr int kLeftIntakeMotorID = 24;
constexpr int kPitchMotorID = 22;
constexpr int kLightSensorID = 0;

//PID Values
constexpr double kPitchP = 0.02;
constexpr double kPitchI = 0.0;
constexpr double kPitchD = 0.0;
constexpr double kPitchFF = 0.0;

constexpr double kMinimumAngle = 0.73;
constexpr double kMaximumAngle = 0.35;

constexpr int kPitchMinOutput = -1;
constexpr int kPitchMaxOutput = 1;

constexpr int kPitchToIntake = 0.38;
constexpr int kPitchL4 = 0.6;

constexpr units::ampere_t kIntakeMotorCurrentLimit = 20_A;
constexpr rev::spark::SparkLowLevel::MotorType kMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
constexpr rev::spark::SparkMaxConfig::IdleMode kIntakeMotorIdleMode = rev::spark::SparkMaxConfig::IdleMode::kBrake;
}


namespace FourBarConstants {
    constexpr int kLeftBarMotorID = 12;
    constexpr int kRightBarMotorID = 17;
    
    constexpr rev::spark::SparkMaxConfig::IdleMode kLeftBarMotorIdleMode =  rev::spark::SparkMaxConfig::IdleMode::kBrake;
    constexpr units::ampere_t kLeftBarMotorCurrentLimit = 40_A;
    constexpr rev::spark::SparkMaxConfig::IdleMode kRightBarMotorIdleMode =  rev::spark::SparkMaxConfig::IdleMode::kBrake;
    constexpr units::ampere_t kRightBarMotorCurrentLimit = 40_A;
    
    constexpr double kLeftBarEncoderPositionFactor = 1;
    constexpr double kLeftBarEncoderVelocityFactor = 1;
    constexpr double kRightBarEncoderPositionFactor = 1;
    constexpr double kRightBarEncoderVelocityFactor = 1;

    constexpr double kFourBarP = 6.0;
    constexpr double kFourBarI = 0.0;
    constexpr double kFourBarD = 0.0;
    constexpr double kFourBarFF = 0.0;
 
    constexpr double kMaximumHeight = 0.277;
    constexpr double kMinimumHeight = 0.498;

    // Maximum outputs that PID can give
    constexpr int kMinimumOutput = -1;
    constexpr int kMaximumOutput = 1;
    constexpr int kFourBarL4 = 0.29;
    constexpr int kFourBarIntake = 0.3;

    constexpr rev::spark::SparkLowLevel::MotorType kMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;

    //constexpr int kFourBarSensA = 0;
    //constexpr int kFourBarSensB = 1;


} // namespace FourBarConstants

namespace ClimberConstants {
    constexpr int kClimberMotorID = 13;
    constexpr rev::spark::SparkLowLevel::MotorType kMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
    constexpr rev::spark::SparkMaxConfig::IdleMode kClimberMotorIdleMode =  rev::spark::SparkMaxConfig::IdleMode::kBrake;
    constexpr units::ampere_t kClimberMotorCurrentLimit = 40_A;

}