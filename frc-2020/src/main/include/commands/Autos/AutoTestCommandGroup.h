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


#include "commands/pathCommands/AutoSwerveFollowPathCommand.h"
#include "commands/pathCommands/WaitForPathToFinishCommand.h"
#include "commands/pathCommands/PathFollowerCommand.h"
#include "commands/pathCommands/PathFollowerShootingCommand.h"


#include "commands/Drive/DriveOpenLoopCommand.h"
#include "commands/Drive/RotateToAngleCommand.h"
#include "commands/Drive/RotateWithTrajectoryCommand.h"

#include "commands/Intake/WaitForBallCountCommand.h"
#include "commands/Intake/RetractIntakeCommand.h"
#include "commands/Intake/ExtendIntakeCommand.h"
#include "commands/Intake/FeederDefaultCommand.h"
#include "commands/Intake/QueFeederCommand.h"
#include "commands/LimeLight/LimeLightRotateToTargetCommand.h"
#include "commands/LimeLight/TurnLimeLightOff.h"
#include "commands/LimeLight/TurnLimeLightOn.h"
#include "commands/LimeLight/LimeLightRotateToTargetCommand.h"
#include "commands/Drive/RotateWithMotionMagic.h"
#include "commands/LimeLight/LimeLightRotateTillOnTarget.h"

#include "Utils/SwerveDrivePathFollower.h"


#include "commands/shooter/SetShooterRangeCommand.h"
#include "commands/shooter/SetShooterSpeedCommand.h"
#include "commands/Shooter/FarShotCommand.h"

#include "subsystems/ShooterSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/FeederSubsystem.h"
#include "subsystems/DriveSubsystem.h"


class AutoTestCommandGroup
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AutoTestCommandGroup> {
 private:
  
 public:
  AutoTestCommandGroup(SwerveDrivePathFollower* m_follower,
                      DriveSubsystem* m_drive,
                      ShooterSubsystem* m_shooter,
                      FeederSubsystem* m_feeder,
                      IntakeSubsystem* m_intake){

    //  double metersToInches = 39.3701;
      std::vector<SwerveDrivePathGenerator::waypoint_t> start;
    start.push_back(SwerveDrivePathGenerator::waypoint_t {0, 0, 0, 0, 0});//start
    start.push_back(SwerveDrivePathGenerator::waypoint_t {-90, 0, 0, 100, 0});
    start.push_back(SwerveDrivePathGenerator::waypoint_t {-180, 0, 0, 0, 0});//pick up ball 4 and 5
    std::vector<SwerveDrivePathGenerator::waypoint_t> tempWaypoints;//if meters
    

    AddCommands(
          PathFollowerShootingCommand(m_drive, m_shooter, m_feeder, 1, start, "start path" ,true)//head to end
    );
  }
};
