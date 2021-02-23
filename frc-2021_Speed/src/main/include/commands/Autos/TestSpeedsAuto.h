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
#include <frc/SmartDashboard/SmartDashboard.h>

class TestSpeedsAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 TestSpeedsAuto> {
 private:
  
 public:
  TestSpeedsAuto(SwerveDrivePathFollower* m_follower,
                      DriveSubsystem* m_drive){

     double metersToInches = 39.3701;
    //  double distToAccel = (std::pow(RobotParameters::k_maxSpeed,2)/(2*RobotParameters::k_maxAccel)) * metersToInches;
    //  double distToDeccel = fabs((std::pow(RobotParameters::k_maxSpeed,2)/(2*RobotParameters::k_maxDeccel)) * metersToInches);
    std::vector<SwerveDrivePathGenerator::waypoint_t> start;
    start.push_back(SwerveDrivePathGenerator::waypoint_t {0, 0, 90, 0, 0});//start
    start.push_back(SwerveDrivePathGenerator::waypoint_t {100, 0, 90, RobotParameters::k_maxSpeed * metersToInches, 0});
    start.push_back(SwerveDrivePathGenerator::waypoint_t {330, 0, 90, RobotParameters::k_maxSpeed * metersToInches, 0});
    start.push_back(SwerveDrivePathGenerator::waypoint_t {500, 0, 90, 0, 0});//pick up ball 4 and 5

    std::vector<SwerveDrivePathGenerator::waypoint_t> tempWaypoints;//if meters
    AddCommands(
      PathFollowerCommand(m_drive, start, "start path" ,true)
    );
  }
};
