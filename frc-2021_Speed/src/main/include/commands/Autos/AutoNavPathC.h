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
class AutoNavPathC
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AutoNavPathC> {
 private:
  double metersToInches = 39.3701;
  double curveSmall = 17;//half of robot(12) + half of cone(2.5) + buffer(2.5)
  double curveBig = 25;
 public:
  AutoNavPathC(DriveSubsystem* m_drive){

    std::vector<SwerveDrivePathGenerator::waypoint_t> one;
    one.push_back(SwerveDrivePathGenerator::waypoint_t {60 + RobotParameters::k_wheelBase*metersToInches/2, 100, 0, 0, 0});//start
    one.push_back(SwerveDrivePathGenerator::waypoint_t {60 + RobotParameters::k_wheelBase*metersToInches/2 + curveBig, 100 + curveBig, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//clear start zone
    one.push_back(SwerveDrivePathGenerator::waypoint_t {90, 150 + curveBig, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//bounce point 1

    std::vector<SwerveDrivePathGenerator::waypoint_t> two;
    two.push_back(SwerveDrivePathGenerator::waypoint_t {90, 150, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//bounce point 1
    two.push_back(SwerveDrivePathGenerator::waypoint_t {130 - curveBig, 60 - curveBig, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//curve around d5
    two.push_back(SwerveDrivePathGenerator::waypoint_t {130 + curveSmall, 60 - curveSmall, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//curve around d5
    two.push_back(SwerveDrivePathGenerator::waypoint_t {180, 150 + curveBig, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//bounce point 2

    std::vector<SwerveDrivePathGenerator::waypoint_t> three;
    three.push_back(SwerveDrivePathGenerator::waypoint_t {180, 150, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//bounce point 2
    three.push_back(SwerveDrivePathGenerator::waypoint_t {210 - curveBig, 60 - curveBig, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//curve around d7
    three.push_back(SwerveDrivePathGenerator::waypoint_t {240 + curveSmall, 60 - curveSmall, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//curve around d8
    three.push_back(SwerveDrivePathGenerator::waypoint_t {270, 150 + curveBig, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//bounce point 3

    std::vector<SwerveDrivePathGenerator::waypoint_t> four;
    four.push_back(SwerveDrivePathGenerator::waypoint_t {270, 150, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//bounce point 3
    four.push_back(SwerveDrivePathGenerator::waypoint_t {300 - curveBig, 120 - curveBig, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//head to end
    four.push_back(SwerveDrivePathGenerator::waypoint_t {300, 100, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//curve into end
    four.push_back(SwerveDrivePathGenerator::waypoint_t {360, 100, 0, 0, 0});//decelerate

    std::vector<SwerveDrivePathGenerator::waypoint_t> tempWaypoints;//if meters
    
    AddCommands(
      //Break Into Multiple Commands
        PathFollowerCommand(m_drive, one, "path path" ,true,false),
        PathFollowerCommand(m_drive, two, "path path" ,true,false),
        PathFollowerCommand(m_drive, three, "path path" ,true,false),
        PathFollowerCommand(m_drive, four, "path path" ,true,false),
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