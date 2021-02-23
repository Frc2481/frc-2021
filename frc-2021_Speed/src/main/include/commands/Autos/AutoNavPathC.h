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

class AutoNavPathC
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AutoNavPathC> {
 private:
  double metersToInches = 39.3701;
  double radCurve = 17; //half of robot(12) + half of cone(2.5) + buffer(2.5)
 public:
  AutoNavPathC(SwerveDrivePathFollower* m_follower,
                      DriveSubsystem* m_drive){

    std::vector<SwerveDrivePathGenerator::waypoint_t> path;
    path.push_back(SwerveDrivePathGenerator::waypoint_t {60 + RobotParameters::k_wheelBase*metersToInches/2, 100, 0, 0, 0});//start
    path.push_back(SwerveDrivePathGenerator::waypoint_t {60 + RobotParameters::k_wheelBase*metersToInches/2 + radCurve, 100 + radCurve, 0, RobotParameters::k_maxSpeed*metersToInches, radCurve});//clear start zone
    path.push_back(SwerveDrivePathGenerator::waypoint_t {90, 150, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//bounce point 1
    path.push_back(SwerveDrivePathGenerator::waypoint_t {130, 60, 0, RobotParameters::k_maxSpeed*metersToInches, radCurve});//curve around d5
    path.push_back(SwerveDrivePathGenerator::waypoint_t {150, 60 -radCurve, 0, RobotParameters::k_maxSpeed*metersToInches, radCurve});//curve around d5
    path.push_back(SwerveDrivePathGenerator::waypoint_t {170, 60, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//curve around d5
    path.push_back(SwerveDrivePathGenerator::waypoint_t {180, 150, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//bounce point 2
    path.push_back(SwerveDrivePathGenerator::waypoint_t {190, 60, 0, RobotParameters::k_maxSpeed*metersToInches, radCurve});//curve around d7
    path.push_back(SwerveDrivePathGenerator::waypoint_t {210, 60 - radCurve, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//curve around d7
    path.push_back(SwerveDrivePathGenerator::waypoint_t {240 - radCurve, 60, 0, RobotParameters::k_maxSpeed*metersToInches, radCurve});//curve around d8
    path.push_back(SwerveDrivePathGenerator::waypoint_t {260, 60, 0, RobotParameters::k_maxSpeed*metersToInches, radCurve});//curve around d8
    path.push_back(SwerveDrivePathGenerator::waypoint_t {270, 150, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//bounce point 3
    path.push_back(SwerveDrivePathGenerator::waypoint_t {280, 110, 0, RobotParameters::k_maxSpeed*metersToInches, radCurve});//head to end
    path.push_back(SwerveDrivePathGenerator::waypoint_t {300, 100, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//curve into end
    path.push_back(SwerveDrivePathGenerator::waypoint_t {360, 100, 0, 0, 0});//decelerate

    std::vector<SwerveDrivePathGenerator::waypoint_t> tempWaypoints;//if meters
    
    AddCommands(
        PathFollowerCommand(m_drive, path, "path path" ,true)
    );
  }
};
