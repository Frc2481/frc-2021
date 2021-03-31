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


#include "subsystems/ShooterSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/FeederSubsystem.h"
#include "subsystems/DriveSubsystem.h"


class AutoLeftCommandGroupStrafe
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AutoLeftCommandGroupStrafe> {
 private:
  
 public:
  AutoLeftCommandGroupStrafe(SwerveDrivePathFollower* m_follower,
                      DriveSubsystem* m_drive,
                      ShooterSubsystem* m_shooter,
                      FeederSubsystem* m_feeder,
                      IntakeSubsystem* m_intake){

    double metersToInches = 39.3701;
    std::vector<SwerveDrivePathGenerator::waypoint_t> one;//pick up ball 4 - 5
    one.push_back(SwerveDrivePathGenerator::waypoint_t {RobotParameters::k_robotLength/2.0, 120 + RobotParameters::k_wheelBase/2.0, 90, 0, 0});//start pos
    one.push_back(SwerveDrivePathGenerator::waypoint_t {56, 206, 17, RobotParameters::k_maxSpeed*metersToInches, 10000});
    one.push_back(SwerveDrivePathGenerator::waypoint_t {103.315, 246.906, 17, 0, 0});

    std::vector<SwerveDrivePathGenerator::waypoint_t> two;//Shoot ball 1 - 5
    two.push_back(SwerveDrivePathGenerator::waypoint_t {104.192, 244.046, 17, 0, 0});
    two.push_back(SwerveDrivePathGenerator::waypoint_t {89.192, 220.546, 90, RobotParameters::k_maxSpeed*metersToInches, 0});
    two.push_back(SwerveDrivePathGenerator::waypoint_t {77.2, 222.756, 180, 0, 0});

    std::vector<SwerveDrivePathGenerator::waypoint_t> three;//pick up ball 6 - 8
    three.push_back(SwerveDrivePathGenerator::waypoint_t {77.2, 222.756, 188, 0, 0});
    three.push_back(SwerveDrivePathGenerator::waypoint_t {36, 189, 90, RobotParameters::k_maxSpeed*metersToInches, 0});
    three.push_back(SwerveDrivePathGenerator::waypoint_t {32, 320, 90, 0, 0});
    
    std::vector<SwerveDrivePathGenerator::waypoint_t> four;//shoot ball 6 - 8
    four.push_back(SwerveDrivePathGenerator::waypoint_t {32, 320, 90, 0, 0});
    four.push_back(SwerveDrivePathGenerator::waypoint_t {54, 249, 130, RobotParameters::k_maxSpeed*metersToInches, 0});
    four.push_back(SwerveDrivePathGenerator::waypoint_t {77, 240, 189, 0, 0});

    std::vector<SwerveDrivePathGenerator::waypoint_t> tempWaypoints;
    

    AddCommands(
      // frc2::ParallelRaceGroup{
      //   FeederDefaultCommand(m_feeder),
      //   frc2::SequentialCommandGroup{
      //     ExtendIntakeCommand(m_intake),
      //     PathFollowerCommand(m_drive, one, "one" ,true),
      //     frc2::ParallelCommandGroup{
      //       PathFollowerShootingCommand(m_drive, two, "two"),
      //       RetractIntakeCommand(m_intake)
      //     },
      // },

      // frc2::ParallelRaceGroup{
      //   FeederDefaultCommand(m_feeder),
      //   frc2::SequentialCommandGroup{
      //     ExtendIntakeCommand(m_intake),
      //     PathFollowerCommand(m_drive, one, "start path" ,true),
      //     frc2::ParallelCommandGroup{
      //       PathFollowerCommand(m_drive, two, "first path"),
      //       RetractIntakeCommand(m_intake)
      //     },
      //     StartShooterCommand(m_shooter),
      //     frc2::ParallelCommandGroup{
      //       SetShooterSpeedCommand(m_shooter,ShooterConstants::kDefaultShooterShortSpeed),
      //       RotateWithMotionMagic(m_drive, 97,1, true)            
      //     }
      //   }
      // },
      
      // frc2::ParallelRaceGroup{
      //   ShootBallCommand(m_shooter, m_feeder),
      //   frc2::WaitCommand(2.5_s)
      // },
      
      // frc2::ParallelRaceGroup{
      //   FeederDefaultCommand(m_feeder),
      //   frc2::SequentialCommandGroup{
      //     frc2::ParallelCommandGroup{
      //       StopShooterCommand(m_shooter),
      //       ExtendIntakeCommand(m_intake),
      //       PathFollowerCommand(m_drive, second,"balls 6-8"),
      //     },
      //     frc2::ParallelCommandGroup{
      //       RetractIntakeCommand(m_intake),
      //       PathFollowerCommand(m_drive, third,"shoot the final part"),
      //     },
          
      //   }
      // },
      // StartShooterCommand(m_shooter),
      //   frc2::ParallelCommandGroup{
      //     frc2::ParallelRaceGroup{
      //       QueFeederCommand(m_feeder,FeederConstants::kShootingFeederSpeed),
      //       RotateWithMotionMagic(m_drive, 97, 1, true)
      //     },
      //     SetShooterSpeedCommand(m_shooter,ShooterConstants::kDefaultShooterShortSpeed)
      //   },
      // frc2::ParallelRaceGroup{
      //   ShootBallCommand(m_shooter, m_feeder),
      //   frc2::WaitCommand(3_s)
      // },
      
      // StopShooterCommand(m_shooter)
    );
  }
};
