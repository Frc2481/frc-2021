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

class AutoNavPathA
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AutoNavPathA> {
 private:
  double metersToInches = 39.3701;
  double radCurve = 17; //half of robot(12) + half of cone(2.5) + buffer(2.5)
 public:
  AutoNavPathA(SwerveDrivePathFollower* m_follower,
                      DriveSubsystem* m_drive){

    std::vector<SwerveDrivePathGenerator::waypoint_t> path;
    path.push_back(SwerveDrivePathGenerator::waypoint_t {60 + RobotParameters::k_wheelBase*metersToInches/2, 80, 90, 0, 0});//start path
    path.push_back(SwerveDrivePathGenerator::waypoint_t {150, 60 + radCurve, 90, RobotParameters::k_maxSpeed*metersToInches, 0});//cone 1, point 1
    path.push_back(SwerveDrivePathGenerator::waypoint_t {150 + radCurve, 60, 90, RobotParameters::k_maxSpeed*metersToInches, radCurve});//cone 1, point 2
    path.push_back(SwerveDrivePathGenerator::waypoint_t {150, 60 - radCurve, 90, RobotParameters::k_maxSpeed*metersToInches, radCurve});//cone 1, point 3
    path.push_back(SwerveDrivePathGenerator::waypoint_t {150 - radCurve, 60, 90, RobotParameters::k_maxSpeed*metersToInches, radCurve});//cone 1, point 4
    path.push_back(SwerveDrivePathGenerator::waypoint_t {150, 60 + radCurve, 90, RobotParameters::k_maxSpeed*metersToInches, radCurve});//cone 1, point 5/1
    path.push_back(SwerveDrivePathGenerator::waypoint_t {240, 120 + radCurve, 90, RobotParameters::k_maxSpeed*metersToInches, 0});//cone 2, point 1
    path.push_back(SwerveDrivePathGenerator::waypoint_t {240 + radCurve, 120, 90, RobotParameters::k_maxSpeed*metersToInches, radCurve});//cone 2, point 2
    path.push_back(SwerveDrivePathGenerator::waypoint_t {240, 120 - radCurve, 90, RobotParameters::k_maxSpeed*metersToInches, radCurve});//cone 2, point 3
    path.push_back(SwerveDrivePathGenerator::waypoint_t {240 - radCurve, 120, 90, RobotParameters::k_maxSpeed*metersToInches, radCurve});//cone 2, point 4
    path.push_back(SwerveDrivePathGenerator::waypoint_t {240, 120 + radCurve, 90, RobotParameters::k_maxSpeed*metersToInches, radCurve});//cone 2, point 5/1
    path.push_back(SwerveDrivePathGenerator::waypoint_t {300, 60 + radCurve, 90, RobotParameters::k_maxSpeed*metersToInches, 0});//cone 3, point 1
    path.push_back(SwerveDrivePathGenerator::waypoint_t {300 + radCurve, 60, 90, RobotParameters::k_maxSpeed*metersToInches, radCurve});//cone 3, point 2
    path.push_back(SwerveDrivePathGenerator::waypoint_t {300, 60 - radCurve, 90, RobotParameters::k_maxSpeed*metersToInches, radCurve});//cone 3, point 3
    path.push_back(SwerveDrivePathGenerator::waypoint_t {300 - radCurve, 60, 90, RobotParameters::k_maxSpeed*metersToInches, radCurve});//cone 3, point 4
    path.push_back(SwerveDrivePathGenerator::waypoint_t {300, 60 + radCurve, 90, RobotParameters::k_maxSpeed*metersToInches, radCurve});//cone 3, point 5/1
    path.push_back(SwerveDrivePathGenerator::waypoint_t {60, 60 + radCurve, 90, RobotParameters::k_maxSpeed*metersToInches, 0});//head to end
    path.push_back(SwerveDrivePathGenerator::waypoint_t {0, 60 + radCurve, 90, 0, 0});

    std::vector<SwerveDrivePathGenerator::waypoint_t> tempWaypoints;//if meters
    
    AddCommands(
        PathFollowerCommand(m_drive, path, "path path" ,true)
    );
  }
};
