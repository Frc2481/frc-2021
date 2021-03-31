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
#include <frc2/command/InstantCommand.h>

#include "commands/pathCommands/PathFollowerCommand.h"


#include "commands/Drive/DriveOpenLoopCommand.h"

#include "commands/Drive/RotateWithMotionMagic.h"

#include "Utils/SwerveDrivePathFollower.h"

#include "subsystems/DriveSubsystem.h"
#include "commands/Drive/EngageBakeCommand.h"
#include "commands/Drive/CheckInRangeCommand.h"
class AutoNavPathC
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AutoNavPathC> {
 public:
  AutoNavPathC(DriveSubsystem* m_drive, SwerveDrivePathFollower m_follower[]){

    AddCommands(
      frc2::ParallelRaceGroup{
        PathFollowerCommand2(m_drive, &m_follower[0], "path path" ,true,false),
        CheckInRangeCommand(m_drive, 133.0, 1)
      },
      frc2::ParallelRaceGroup{
        PathFollowerCommand2(m_drive, &m_follower[1], "path path" ,true,false),
        frc2::SequentialCommandGroup{
          frc2::WaitCommand(1_s),
          CheckInRangeCommand(m_drive, 147.0, 1)//144
        }
        
      },
      frc2::ParallelRaceGroup{
        PathFollowerCommand2(m_drive, &m_follower[2], "path path" ,true,false),
        frc2::SequentialCommandGroup{
          frc2::WaitCommand(2_s),
          CheckInRangeCommand(m_drive, 144.0, 1)//146
        }
      },
      PathFollowerCommand2(m_drive, &m_follower[3], "path path" ,true,false),
      PathFollowerCommand2(m_drive, &m_follower[4], "path path" ,true,false),
      EngageBakeCommand(m_drive)
    );
  }
};




// std::vector<SwerveDrivePathGenerator::waypoint_t> path;
//     path.push_back(SwerveDrivePathGenerator::waypoint_t {60 + RobotParameters::k_wheelBase*metersToInches/2, 100, 0, 0, 0});//start
//     path.push_back(SwerveDrivePathGenerator::waypoint_t {60 + RobotParameters::k_wheelBase*metersToInches/2 + curveBig, 100 + curveBig, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//clear start zone
//     path.push_back(SwerveDrivePathGenerator::waypoint_t {90, 150, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//bounce point 1
//     path.push_back(SwerveDrivePathGenerator::waypoint_t {130 - radCurve, 60 - radCurve, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//curve around d5
//     path.push_back(SwerveDrivePathGenerator::waypoint_t {130 + radCurve, 60 - radCurve, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//curve around d5
//     path.push_back(SwerveDrivePathGenerator::waypoint_t {180, 150, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//bounce point 2
//     path.push_back(SwerveDrivePathGenerator::waypoint_t {210 - radCurve, 60 - radCurve, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//curve around d7
//     path.push_back(SwerveDrivePathGenerator::waypoint_t {240 + radCurve, 60 - radCurve, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//curve around d8
//     path.push_back(SwerveDrivePathGenerator::waypoint_t {270, 150, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//bounce point 3
//     path.push_back(SwerveDrivePathGenerator::waypoint_t {300 - radCurve, 120 - radCurve, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//head to end
//     path.push_back(SwerveDrivePathGenerator::waypoint_t {300, 100, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//curve into end
//     path.push_back(SwerveDrivePathGenerator::waypoint_t {360, 100, 0, 0, 0});//decelerate