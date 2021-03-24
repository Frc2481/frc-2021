/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
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
    static constexpr int kFrontLeftTurningMotorID = 1;
    static constexpr int kFrontRightTurningMotorID = 2;
    static constexpr int kBackLeftTurningMotorID = 3;
    static constexpr int kBackRightTurningMotorID = 4;
    static constexpr int kAIntakeID = 9;
    static constexpr int kBIntakeID = 10;   
}

namespace FalconIDs{
    static constexpr int kFrontLeftDriveMotorID = 5;
    static constexpr int kFrontRightDriveMotorID = 6;
    static constexpr int kBackLeftDriveMotorID = 7;
    static constexpr int kBackRightDriveMotorID = 8;

}

namespace DriveConstants {

    constexpr bool kFrontLeftTurningEncoderReversed = true;
    constexpr bool kBackLeftTurningEncoderReversed = true;
    constexpr bool kFrontRightTurningEncoderReversed = true;
    constexpr bool kBackRightTurningEncoderReversed = true;

    constexpr bool kFrontLeftTurningMotorReversed = false;
    constexpr bool kBackLeftTurningMotorReversed = false;
    constexpr bool kFrontRightTurningMotorReversed = false;
    constexpr bool kBackRightTurningMotorReversed = false;

    constexpr bool kFrontLeftDriveEncoderReversed = false;
    constexpr bool kBackLeftDriveEncoderReversed = false;
    constexpr bool kFrontRightDriveEncoderReversed = false;
    constexpr bool kBackRightDriveEncoderReversed = false;

    constexpr bool kGyroReversed = true;

}  // namespace DriveConstants

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

enum class CommonDrive{
    Brake = 0,
    Coast = 1,
    EEPROMSetting = 2
};
namespace IntakeConstants{
    static constexpr double kMaxIntakeAmp = 29.0;
    static constexpr double kAIntakeSpeed = 0.9;
    static constexpr double kAIntakeReverse = -0.6;
    static constexpr double kBIntakeSpeed = 1.0;
    static constexpr double kBIntakeReverse = -0.5;

    static constexpr double kAIntakeCurrent = 25.0;
    static constexpr double kBIntakeCurrent = 25.0;
}
namespace PathConstants{
    static constexpr double kMinLookAhead = 6*.0254;//6
    static constexpr double kMaxLookAhead = RobotParameters::k_maxSpeed * .16255997;//calculated constant based off of last years robot and max look ahead
}
