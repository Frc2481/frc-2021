/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/units.h>
#include <wpi/math>

#include "RobotParameters.h"
#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace TalonIDs{
    static constexpr int kFrontRightTurningMotorID = 3;//1
    static constexpr int kFrontLeftTurningMotorID = 1;//4
    static constexpr int kRearRightTurningMotorID = 4;//3
    static constexpr int kRearLeftTurningMotorID = 2;//2   
}

namespace FalconIDs{
    static constexpr int kFrontRightDriveMotorID = 7;//5
    static constexpr int kFrontLeftDriveMotorID = 5;//6
    static constexpr int kRearRightDriveMotorID = 8;//7
    static constexpr int kRearLeftDriveMotorID = 6;//8

}
namespace VictorIDs{
    static constexpr int kRightIntakeID = 11;
    static constexpr int kLeftIntakeID = 12;
}

namespace BeamBreaks{
    static constexpr int kBeamBreakID = 762;
}
namespace DriveConstants {

constexpr bool kFrontLeftTurningEncoderReversed = true;
constexpr bool kRearLeftTurningEncoderReversed = true;
constexpr bool kFrontRightTurningEncoderReversed = true;
constexpr bool kRearRightTurningEncoderReversed = true;

constexpr bool kFrontLeftTurningMotorReversed = false;
constexpr bool kRearLeftTurningMotorReversed = false;
constexpr bool kFrontRightTurningMotorReversed = false;
constexpr bool kRearRightTurningMotorReversed = false;

constexpr bool kFrontLeftDriveEncoderReversed = false;//true
constexpr bool kRearLeftDriveEncoderReversed = false;//true
constexpr bool kFrontRightDriveEncoderReversed = false;//true
constexpr bool kRearRightDriveEncoderReversed = false;//true

constexpr bool kGyroReversed = true;

}  // namespace DriveConstants

namespace ModuleConstants {
constexpr int kEncoderCPR = 4096;
constexpr double kWheelDiameterMeters = .15;
constexpr double kDriveEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (kWheelDiameterMeters * wpi::math::pi) / static_cast<double>(kEncoderCPR);

constexpr double kTurningEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (wpi::math::pi * 2) / static_cast<double>(kEncoderCPR);

constexpr double kPModuleTurningController = 1;
constexpr double kPModuleDriveController = 1;
}  // namespace ModuleConstants

namespace AutoConstants {
using radians_per_second_squared_t =
    units::compound_unit<units::radians,
                         units::inverse<units::squared<units::second>>>;

constexpr auto kMaxSpeed = units::meters_per_second_t(RobotParameters::k_maxSpeed);
constexpr auto kMaxAcceleration = units::meters_per_second_squared_t(3);
constexpr auto kMaxAngularSpeed = units::radians_per_second_t(3.142);
constexpr auto kMaxAngularAcceleration =
    units::unit_t<radians_per_second_squared_t>(3.142);

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;

}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
}  // namespace OIConstants


enum class CommonModes{
    DutyCycle = 0,
    Velocity = 1,
    Voltage = 2,
    Position = 3,
    SmartMotion = 4,
    Current = 5,
    SmartVelocity = 6,
    PercentOutput = 7,
    Follower = 8,
    MotionProfile = 9,
    MotionMagic = 10,
    MotionProfileArc = 11,
    Disabled = 15
};
namespace LimeLightConstants{
    static constexpr double kLimeLightHeight = 1.09;//ft
    static constexpr double kLimeLightAngle = 20;//deg
    static constexpr double kLimeLightHardWarePanAngle = 12;//deg
    static constexpr double kTargetHeight =  2.03;//m
}
namespace IntakeConstants{
    static constexpr double kDefaultIntakeRollerSpeed = -.7; //TODO: Improve these speeds
}
namespace PathConstants{
    static constexpr double kMinLookAhead = 6*.0254;
    static constexpr double kMaxLookAhead = 24*.0254;
}
