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
#include "commands/pathCommands/PathFollowerCommand2.h"



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


class AutoLeftCommandGroup
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AutoLeftCommandGroup> {
 private:
  
 public:
  AutoLeftCommandGroup(DriveSubsystem* m_drive,
                      ShooterSubsystem* m_shooter,
                      FeederSubsystem* m_feeder,
                      IntakeSubsystem* m_intake,
                      SwerveDrivePathFollower m_follower[]){

    // std::vector<SwerveDrivePathGenerator::waypoint_t> start;
    // start.push_back(SwerveDrivePathGenerator::waypoint_t {RobotParameters::k_robotLength/2.0, 120 + RobotParameters::k_wheelBase/2.0, 90, 0, 0});//start
    // start.push_back(SwerveDrivePathGenerator::waypoint_t {56, 206, 21.57, 1000, 10000});//17deg
    // start.push_back(SwerveDrivePathGenerator::waypoint_t {103.315+30.76, 246.906-8.805+5.5, 21.57, 0, 0});//pick up ball 4 and 5

    // std::vector<SwerveDrivePathGenerator::waypoint_t> first;
    // // first.push_back(SwerveDrivePathGenerator::waypoint_t {101.67+2.39-2.868+3, 231.5-6.577-0.877+20, 17, 0, 0});//pick up ball 4 and 5
    // first.push_back(SwerveDrivePathGenerator::waypoint_t {103.315+30.76, 246.906-8.805+6, 21.57, 0, 0});//pick up ball 4 and 5
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


    AddCommands(
      frc2::ParallelRaceGroup{
        FeederDefaultCommand(m_feeder),
        frc2::SequentialCommandGroup{
          // TurnLimeLightOff(),
          ExtendIntakeCommand(m_intake),
          PathFollowerCommand2(m_drive, &m_follower[0], "start path" ,true),//head to end
          frc2::ParallelCommandGroup{
            PathFollowerCommand2(m_drive, &m_follower[1], "first path"),
            RetractIntakeCommand(m_intake)
          },
          StartShooterCommand(m_shooter),
          frc2::ParallelCommandGroup{
            SetShooterSpeedCommand(m_shooter,ShooterConstants::kDefaultShooterShortSpeed),
            RotateWithMotionMagic(m_drive, 90, .625, true)
            // TurnLimeLightOn(),
            // LimeLightRotateTillOnTarget(m_drive, 97, 2)
            
          }
        }
      },
      
      frc2::ParallelRaceGroup{
        ShootBallCommand(m_shooter, m_feeder),
        frc2::WaitCommand(2.5_s)//2.25
      },
      
      frc2::ParallelRaceGroup{
        FeederDefaultCommand(m_feeder),
        frc2::SequentialCommandGroup{
          frc2::ParallelCommandGroup{
            StopShooterCommand(m_shooter),
            // TurnLimeLightOff(),
            ExtendIntakeCommand(m_intake),
            PathFollowerCommand2(m_drive, &m_follower[2],"balls 6-8"),
          },
          frc2::ParallelCommandGroup{
            RetractIntakeCommand(m_intake),
            PathFollowerCommand2(m_drive, &m_follower[3],"shoot the final part"),
          },
          
        }
      },
      // frc2::WaitCommand(0.5_s),
      StartShooterCommand(m_shooter),
        frc2::ParallelCommandGroup{
          frc2::ParallelRaceGroup{
            QueFeederCommand(m_feeder,FeederConstants::kShootingFeederSpeed),
            RotateWithMotionMagic(m_drive, 97.7, .625, true)//101  //97
          },
          SetShooterSpeedCommand(m_shooter,ShooterConstants::kDefaultShooterShortSpeed)
          // TurnLimeLightOn(),
          // LimeLightRotateTillOnTarget(m_drive, 97, 2)
        },
      frc2::ParallelRaceGroup{
        ShootBallCommand(m_shooter, m_feeder),
        frc2::WaitCommand(3_s)
      },
      
      StopShooterCommand(m_shooter)
          // TurnLimeLightOff()
    );
  }
};
