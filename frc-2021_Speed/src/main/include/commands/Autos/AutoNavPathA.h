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

#include "commands/pathCommands/PathFollowerCommand2.h"


#include "commands/Drive/DriveOpenLoopCommand.h"

#include "commands/Drive/RotateWithMotionMagic.h"

#include "Utils/SwerveDrivePathFollower.h"

#include "subsystems/DriveSubsystem.h"
#include "commands/Drive/EngageBakeCommand.h"


class AutoNavPathA
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AutoNavPathA> {
 private:
  double metersToInches = 39.3701;
  double curveSmall = 17;//half of robot(12) + half of cone(2.5) + buffer(2.5)
  double curveBig = 30;//25
 public:
  AutoNavPathA(DriveSubsystem* m_drive, SwerveDrivePathFollower* follower){
    
    
    AddCommands(

      //Break Into Multiple Commands
        PathFollowerCommand2(m_drive, follower, "path path" ,true),
        EngageBakeCommand(m_drive)
    );
  }
};








// std::vector<SwerveDrivePathGenerator::waypoint_t> one;//RobotParameters::k_maxSpeed*metersToInches
    // path.push_back(SwerveDrivePathGenerator::waypoint_t {60 + RobotParameters::k_wheelBase*metersToInches/2, 80, 0, 0, 0});//start path
    // one.push_back(SwerveDrivePathGenerator::waypoint_t {150 + radCurve, 60 + radCurve, 0, 40, 0});//cone 1, point 1
    // one.push_back(SwerveDrivePathGenerator::waypoint_t {150 + radCurve, 60 - radCurve, 0, 40, 0});//cone 1, point 2
    // one.push_back(SwerveDrivePathGenerator::waypoint_t {150 - radCurve, 60 - radCurve, 0, 40, 0});//cone 1, point 3
    // path.push_back(SwerveDrivePathGenerator::waypoint_t {150 - radCurve, 60 + radCurve, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//cone 1, point 4
    // path.push_back(SwerveDrivePathGenerator::waypoint_t {240 + radCurve, 120 - radCurve, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//cone 2, point 1
    // path.push_back(SwerveDrivePathGenerator::waypoint_t {240 + radCurve, 120 + radCurve, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//cone 2, point 2
    // path.push_back(SwerveDrivePathGenerator::waypoint_t {240 - radCurve, 120 + radCurve, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//cone 2, point 3
    // path.push_back(SwerveDrivePathGenerator::waypoint_t {240 - radCurve, 120 - radCurve, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//cone 2, point 4
    // path.push_back(SwerveDrivePathGenerator::waypoint_t {300 - radCurve, 60 - radCurve, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//cone 3, point 1
    // path.push_back(SwerveDrivePathGenerator::waypoint_t {300 + radCurve, 60 - radCurve, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//cone 3, point 2
    // path.push_back(SwerveDrivePathGenerator::waypoint_t {300 + radCurve, 60 + radCurve, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//cone 3, point 3
    // path.push_back(SwerveDrivePathGenerator::waypoint_t {300 - radCurve, 60 + radCurve, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//cone 3, point 4
    // path.push_back(SwerveDrivePathGenerator::waypoint_t {300, 60 + radCurve, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//cone 3, point 5/1
    // path.push_back(SwerveDrivePathGenerator::waypoint_t {60, 60 + radCurve, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//head to end
    // path.push_back(SwerveDrivePathGenerator::waypoint_t {0, 60 + radCurve, 0, 0, 0});