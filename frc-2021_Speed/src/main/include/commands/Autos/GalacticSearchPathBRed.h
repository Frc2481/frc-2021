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
#include "subsystems/IntakeSubsystem.h"
#include <frc/PowerDistributionPanel.h>

#include "commands/Intake/IntakesDefaultCommand.h"
#include "commands/Intake/DropIntake.h"

class GalacticSearchPathBRed
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 GalacticSearchPathBRed> {
 private:
  
 public:
  GalacticSearchPathBRed(SwerveDrivePathFollower* m_follower,
                      DriveSubsystem* m_drive,
                      IntakeSubsystem* m_intake,
                      frc::PowerDistributionPanel* m_PDP){

     double metersToInches = 39.3701;
    std::vector<SwerveDrivePathGenerator::waypoint_t> path;
    path.push_back(SwerveDrivePathGenerator::waypoint_t {43.5, 120, 0, 0, 0});//start path
    path.push_back(SwerveDrivePathGenerator::waypoint_t {90, 120, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//pick up ball 1
    path.push_back(SwerveDrivePathGenerator::waypoint_t {150, 60, -45, RobotParameters::k_maxSpeed*metersToInches, 0});//pick up ball 2
    path.push_back(SwerveDrivePathGenerator::waypoint_t {210, 120, -45, RobotParameters::k_maxSpeed*metersToInches, 0});//pick up ball 3
    path.push_back(SwerveDrivePathGenerator::waypoint_t {330, 120, -45, RobotParameters::k_maxSpeed*metersToInches, 0});//head to end
    path.push_back(SwerveDrivePathGenerator::waypoint_t {360, 120, -45, 0, 0});//final 30in after endzone

    std::vector<SwerveDrivePathGenerator::waypoint_t> tempWaypoints;//if meters
    
    AddCommands(
      frc2::ParallelCommandGroup{
        DropIntake(m_intake),
        IntakesDefaultCommand(m_intake,m_PDP),
        PathFollowerCommand(m_drive, path, "path path" ,true),
      }
    );
  }
};
