// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// #pragma once

// #include <frc2/command/CommandHelper.h>
// #include <frc2/command/SequentialCommandGroup.h>
// #include <frc2/command/WaitCommand.h>
// #include <frc2/command/ParallelCommandGroup.h>
// #include <frc2/command/ParallelRaceGroup.h>
// #include "commands/pathCommands/AutoSwerveFollowPathCommand.h"
// #include "commands/pathCommands/WaitForPathToFinishCommand.h"
// #include "commands/pathCommands/PathFollowerCommand.h"
// #include "commands/pathCommands/StartPathFollowing.h"
// #include "commands/pathCommands/StopPathFollowing.h"
// #include "commands/pathCommands/WaitForPosCommand.h"
// #include "commands/pathCommands/FollowPathToPosCommandGroup.h"
// #include "commands/DriveOpenLoopCommand.h"
// #include "commands/WaitForBallCountCommand.h"
// #include "subsystems/DriveSubsystem.h"
// #include "Utils/SwerveDrivePathFollower.h"
// #include "subsystems/ShooterSubsystem.h"
// #include "subsystems/IntakeSubsystem.h"
// #include "subsystems/FeederSubsystem.h"
// #include "subsystems/IndexerSubsystem.h"
// #include "commands/LimeLightRotateToTargetCommand.h"

// #include "commands/SetShooterRangeCommand.h"
// #include "commands/SetShooterSpeedCommand.h"
// #include "commands/ExtendIntakeCommand.h"
// #include "commands/RetractIntakeCommand.h"
// #include <frc2/command/WaitCommand.h>
// #include "commands/LimeLightRotateToTargetCommand.h"
// class AutoRightCommandGroup
//     : public frc2::CommandHelper<frc2::SequentialCommandGroup,
//                                  AutoRightCommandGroup> {
//  private:
  
//  public:
//   AutoRightCommandGroup(SwerveDrivePathFollower* m_follower,
//                       DriveSubsystem* m_drive,
//                       ShooterSubsystem* m_shooter,
//                       FeederSubsystem* m_feeder,
//                       IntakeSubsystem* m_intake
//                       ){

//     //  double metersToInches = 39.3701;
//     std::vector<SwerveDrivePathGenerator::waypoint_t> waypoints;
//     waypoints.push_back(SwerveDrivePathGenerator::waypoint_t {10, 20, 0, 0, 0});//start at
//     waypoints.push_back(SwerveDrivePathGenerator::waypoint_t {18, 23, 0, RobotParameters::k_maxSpeed, 0});//to pick up
//     waypoints.push_back(SwerveDrivePathGenerator::waypoint_t {20, 25, 0, RobotParameters::k_maxSpeed/3, 0});//pick up to point
//     waypoints.push_back(SwerveDrivePathGenerator::waypoint_t {14, 15, 90, RobotParameters::k_maxSpeed, 0});//to shoot point
//     waypoints.push_back(SwerveDrivePathGenerator::waypoint_t {14, 19, 120, RobotParameters::k_maxSpeed, 0});//to next pick up point
//     waypoints.push_back(SwerveDrivePathGenerator::waypoint_t {11, 18, -160, RobotParameters::k_maxSpeed/3, 0});//pick up to point
//     waypoints.push_back(SwerveDrivePathGenerator::waypoint_t {14, 15, 180, RobotParameters::k_maxSpeed, 0});//to shoot point
//     waypoints.push_back(SwerveDrivePathGenerator::waypoint_t {40, 20, 0, 0, 0});//end at
//     std::vector<SwerveDrivePathGenerator::waypoint_t> tempWaypoints;

//     CoordinateTranslation::WPILibToNolan(waypoints, tempWaypoints);

    
//     // workaround of vel and accel zero at start

//     m_follower->generatePath(tempWaypoints);
    
//     // m_follower.start(waypoints);
//     AddCommands(
//       StartPathFollowing(m_follower, m_drive),//start
//       StartShooterCommand(m_shooter),
//       SetShooterSpeedCommand(m_shooter,.2),
//       SetShooterRangeCommand(m_shooter, false),
//       FollowPathToPosCommandGroup(m_follower, m_drive, waypoints[1].xPos, waypoints[1].yPos, .1, .1),//intake at range
//       ExtendIntakeCommand(m_intake),
//       FollowPathToPosCommandGroup(m_follower, m_drive, waypoints[2].xPos, waypoints[2].yPos, .05, .05),
//       RetractIntakeCommand(m_intake),
//       FollowPathToPosCommandGroup(m_follower, m_drive, waypoints[3].xPos, waypoints[3].yPos, .1, .1),
//       frc2::ParallelCommandGroup{//go to shoot speed and rotate to target
//           SetShooterSpeedCommand(m_shooter,ShooterConstants::),//shoot
//           LimeLightRotateToTargetCommand(m_drive,.1)
//       },
      
//       frc2::ParallelRaceGroup{
//         ShootBallCommand(m_shooter, m_feeder),
//         frc2::WaitCommand(2_s)//TODO find correct time
//       },

//       FollowPathToPosCommandGroup(m_follower, m_drive, waypoints[4].xPos, waypoints[4].yPos, .1, .1),//intake at range
//       ExtendIntakeCommand(m_intake),
//       FollowPathToPosCommandGroup(m_follower, m_drive, waypoints[5].xPos, waypoints[5].yPos, .05, .05),
//       RetractIntakeCommand(m_intake),
//       FollowPathToPosCommandGroup(m_follower, m_drive, waypoints[6].xPos, waypoints[6].yPos, .1, .1),

//       frc2::ParallelCommandGroup{//go to shoot speed and rotate to target
//           SetShooterSpeedCommand(m_shooter,ShooterConstants::),//shoot
//           LimeLightRotateToTargetCommand(m_drive,.1)
//       },

//       frc2::ParallelRaceGroup{
//         ShootBallCommand(m_shooter, m_feeder),
//         frc2::WaitCommand(2_s)//TODO find correct time
//       },
//       PathFollowerCommand(m_follower,m_drive),//head to end
//       StopPathFollowing(m_follower, m_drive)
//     );
//   }
// };
