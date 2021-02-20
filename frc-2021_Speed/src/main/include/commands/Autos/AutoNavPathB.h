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
#include "commands/Drive/RotateToAngleCommand.h"

#include "commands/Drive/RotateWithMotionMagic.h"

#include "Utils/SwerveDrivePathFollower.h"

#include "subsystems/DriveSubsystem.h"

class AutoNavPathB
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AutoNavPathB> {
 private:
  double metersToInches = 39.3701;
  double radCurve = 17; //half of robot(12) + half of cone(2.5) + buffer(2.5)
 public:
  AutoNavPathB(SwerveDrivePathFollower* m_follower,
                      DriveSubsystem* m_drive){

    std::vector<SwerveDrivePathGenerator::waypoint_t> path;
    path.push_back(SwerveDrivePathGenerator::waypoint_t {60 + RobotParameters::k_wheelBase*metersToInches/2, 40, 90, 0, 0});//start
    path.push_back(SwerveDrivePathGenerator::waypoint_t {120, 60 + radCurve, 90, RobotParameters::k_maxSpeed*metersToInches, 0});//over cone 1
    path.push_back(SwerveDrivePathGenerator::waypoint_t {240, 60 + radCurve, 90, RobotParameters::k_maxSpeed*metersToInches, 0});//over cone 5
    path.push_back(SwerveDrivePathGenerator::waypoint_t {270, 60, 90, RobotParameters::k_maxSpeed*metersToInches, 0});//between cones 5 and 6
    path.push_back(SwerveDrivePathGenerator::waypoint_t {300, 60 - radCurve, 90, RobotParameters::k_maxSpeed*metersToInches, radCurve});//under cone 6
    path.push_back(SwerveDrivePathGenerator::waypoint_t {300 + radCurve, 60, 90, RobotParameters::k_maxSpeed*metersToInches, radCurve});//right of cone 6
    path.push_back(SwerveDrivePathGenerator::waypoint_t {300, 60 + radCurve, 90, RobotParameters::k_maxSpeed*metersToInches, radCurve});//over cone 6
    path.push_back(SwerveDrivePathGenerator::waypoint_t {270, 60, 90, RobotParameters::k_maxSpeed*metersToInches, radCurve});//between cones 5 and 6
    path.push_back(SwerveDrivePathGenerator::waypoint_t {240, 60 - radCurve, 90, RobotParameters::k_maxSpeed*metersToInches, 0});//under cone 5
    path.push_back(SwerveDrivePathGenerator::waypoint_t {120, 60 - radCurve, 90, RobotParameters::k_maxSpeed*metersToInches, 0});//under cone 1
    path.push_back(SwerveDrivePathGenerator::waypoint_t {60 + RobotParameters::k_wheelBase*metersToInches/2, 60, 90, RobotParameters::k_maxSpeed*metersToInches, 0});//head to end
    path.push_back(SwerveDrivePathGenerator::waypoint_t {0, 60 + radCurve, 90, 0, 0});//decelerate

    std::vector<SwerveDrivePathGenerator::waypoint_t> tempWaypoints;//if meters
    
    AddCommands(
        PathFollowerCommand(m_drive, path, "path path" ,true)
    );
  }
};
