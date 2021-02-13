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


class AutoLeftCommandGroup
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AutoLeftCommandGroup> {
 private:
  
 public:
  AutoLeftCommandGroup(SwerveDrivePathFollower* m_follower,
                      DriveSubsystem* m_drive){

    //  double metersToInches = 39.3701;
    std::vector<SwerveDrivePathGenerator::waypoint_t> start;
    start.push_back(SwerveDrivePathGenerator::waypoint_t {0, 0, 90, 0, 0});//start
    start.push_back(SwerveDrivePathGenerator::waypoint_t {90, 0, 90, 288, 0});
    start.push_back(SwerveDrivePathGenerator::waypoint_t {120, 0, 90, 0, 0});//pick up ball 4 and 5

    // std::vector<SwerveDrivePathGenerator::waypoint_t> first;
    // first.push_back(SwerveDrivePathGenerator::waypoint_t {101.67+2.39-2.868+3, 231.5-6.577-0.877+20, 17, 0, 0});//pick up ball 4 and 5
    // first.push_back(SwerveDrivePathGenerator::waypoint_t {101.67+2.39-2.868-12, 231.5-6.577-0.877-3.5, 90, RobotParameters::k_maxSpeed*39.38, 0});//pick up ball 4 and 5
    // first.push_back(SwerveDrivePathGenerator::waypoint_t {77.2, 231.5-6.577-0.877-7.01+2.86+2.86, 180, 0, 0});//pick up ball 4 and 5

    // std::vector<SwerveDrivePathGenerator::waypoint_t> second;
    // second.push_back(SwerveDrivePathGenerator::waypoint_t {77.2, 231.5-6.577-0.877-7.01+2.86+2.86, 188, 0, 0});//pick up ball 4 and 5 77.2, 217.04
    // second.push_back(SwerveDrivePathGenerator::waypoint_t {36, 189, 90, (RobotParameters::k_maxSpeed)*39.38, 0}); // move
    // second.push_back(SwerveDrivePathGenerator::waypoint_t {32, 320, 90, 0, 0}); // shoot
    
    // std::vector<SwerveDrivePathGenerator::waypoint_t> third;
    // third.push_back(SwerveDrivePathGenerator::waypoint_t {32, 320, 90, 0, 0}); // move
    // third.push_back(SwerveDrivePathGenerator::waypoint_t {54, 249, 130, (RobotParameters::k_maxSpeed)*39.38, 0}); // other end
    // third.push_back(SwerveDrivePathGenerator::waypoint_t {77, 240, 189, 0, 0}); // other end
    //101.67+2.39-2.868-23.99-.877-.877, 222.756
    std::vector<SwerveDrivePathGenerator::waypoint_t> tempWaypoints;//if meters
    
    AddCommands(
      PathFollowerCommand(m_drive, start, "start path" ,true),
      frc2::WaitCommand(2_s)
      // PathFollowerCommand(m_drive, first, "first path"),
      // RotateWithMotionMagic(m_drive, 97,1),
      // frc2::WaitCommand(2_s),
      // PathFollowerCommand(m_drive, second,"balls 6-8"),
      // frc2::WaitCommand(2_s),
      // PathFollowerCommand(m_drive, third,"shoot the final part"),
      // RotateWithMotionMagic(m_drive, 97, 1)
    );
  }
};
