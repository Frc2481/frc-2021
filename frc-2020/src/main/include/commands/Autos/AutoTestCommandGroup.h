/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include "commands/pathCommands/AutoSwerveFollowPathCommand.h"
#include "commands/pathCommands/WaitForPathToFinishCommand.h"
#include "commands/pathCommands/PathFollowerCommand.h"
#include "commands/pathCommands/StartPathFollowing.h"
#include "commands/pathCommands/StopPathFollowing.h"
#include "commands/pathCommands/WaitForPosCommand.h"
#include "commands/pathCommands/FollowPathToPosCommandGroup.h"
#include "commands/DriveOpenLoopCommand.h"
#include "commands/WaitForBallCountCommand.h"
#include "subsystems/DriveSubsystem.h"
#include "Utils/SwerveDrivePathFollower.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/FeederSubsystem.h"
#include "commands/LimeLight/LimeLightRotateToTargetCommand.h"
#include "commands/shooter/SetShooterRangeCommand.h"
#include "commands/shooter/SetShooterSpeedCommand.h"
#include <frc2/command/WaitCommand.h>
#include "commands/RetractIntakeCommand.h"
#include "commands/ExtendIntakeCommand.h"
#include "commands/LimeLight/LimeLightRotateToTargetCommand.h"
class AutoTestCommandGroup
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AutoTestCommandGroup> {
 private:
  
 public:
  AutoTestCommandGroup(SwerveDrivePathFollower* m_follower,
                      DriveSubsystem* m_drive,
                      ShooterSubsystem* m_shooter,
                      FeederSubsystem* m_feeder,
                      IntakeSubsystem* m_intake){

    //  double metersToInches = 39.3701;
    std::vector<SwerveDrivePathGenerator::waypoint_t> waypoints;
    waypoints.push_back(SwerveDrivePathGenerator::waypoint_t {0, 0, 90, 0, 0});//start
    waypoints.push_back(SwerveDrivePathGenerator::waypoint_t {0, 100, 180, 100000, 0});//
    waypoints.push_back(SwerveDrivePathGenerator::waypoint_t {0, 200, -90, 100000, 0});//
    waypoints.push_back(SwerveDrivePathGenerator::waypoint_t {0, 300, 0, 100000, 0});
    waypoints.push_back(SwerveDrivePathGenerator::waypoint_t {0, 400, 90, 100000, 0});
    std::vector<SwerveDrivePathGenerator::waypoint_t> tempWaypoints;

    // CoordinateTranslation::WPILibToNolan(waypoints, tempWaypoints);

    
    // workaround of vel and accel zero at start

    // m_follower->generatePath(Waypoints);
    
    // m_follower.start(waypoints);
    AddCommands(
        PathFollowerCommand(m_drive, waypoints, "test", true)
    //   StartPathFollowing(m_follower, m_drive),//start
    //   PathFollowerCommand(m_follower,m_drive),//head to end
    //   StopPathFollowing(m_follower, m_drive)
    );
  }
};
