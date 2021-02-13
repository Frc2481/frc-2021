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

#include "Utils/SwerveDrivePathFollower.h"


#include "commands/shooter/SetShooterRangeCommand.h"
#include "commands/shooter/SetShooterSpeedCommand.h"


#include "subsystems/ShooterSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/FeederSubsystem.h"
#include "subsystems/DriveSubsystem.h"


class FordAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 FordAuto> {
 private:
  
 public:
  FordAuto(SwerveDrivePathFollower* m_follower,
                      DriveSubsystem* m_drive,
                      ShooterSubsystem* m_shooter,
                      FeederSubsystem* m_feeder,
                      IntakeSubsystem* m_intake){

     double metersToInches = 39.3701;
    std::vector<SwerveDrivePathGenerator::waypoint_t> one;
    one.push_back(SwerveDrivePathGenerator::waypoint_t {190, 120, 180, 0, 0});//start
    one.push_back(SwerveDrivePathGenerator::waypoint_t {190, 135, 162, RobotParameters::k_maxSpeed*metersToInches,0});
    one.push_back(SwerveDrivePathGenerator::waypoint_t {190, 246, 162, 0, 0});//shoot 1 - 3

    std::vector<SwerveDrivePathGenerator::waypoint_t> two;
    two.push_back(SwerveDrivePathGenerator::waypoint_t {190, 246, 162, 0, 0});//start
    two.push_back(SwerveDrivePathGenerator::waypoint_t {190.68/2, 247.88/2, 162, RobotParameters::k_maxSpeed*metersToInches,0});
    two.push_back(SwerveDrivePathGenerator::waypoint_t {190.68, 247.88, 162, 0, 0});//shoot 1 - 3
    // std::vector<SwerveDrivePathGenerator::waypoint_t> two;
    // two.push_back(SwerveDrivePathGenerator::waypoint_t {156, 240, 180, 0, 0});//pick up ball 4 and 5
    // two.push_back(SwerveDrivePathGenerator::waypoint_t {101.67+2.39-2.868-12, 231.5-6.577-0.877-3.5, 90, RobotParameters::k_maxSpeed*39.38, 0});//pick up ball 4 and 5
    // two.push_back(SwerveDrivePathGenerator::waypoint_t {77.2, 231.5-6.577-0.877-7.01+2.86+2.86, 180, 0, 0});//pick up ball 4 and 5

    // std::vector<SwerveDrivePathGenerator::waypoint_t> three;
    // three.push_back(SwerveDrivePathGenerator::waypoint_t {77.2, 231.5-6.577-0.877-7.01+2.86+2.86, 188, 0, 0});//pick up ball 4 and 5 77.2, 217.04
    // three.push_back(SwerveDrivePathGenerator::waypoint_t {36, 189, 90, (RobotParameters::k_maxSpeed)*39.38, 0}); // move
    // three.push_back(SwerveDrivePathGenerator::waypoint_t {32, 320, 90, 0, 0}); // shoot
    
    // std::vector<SwerveDrivePathGenerator::waypoint_t> four;
    // four.push_back(SwerveDrivePathGenerator::waypoint_t {32, 320, 90, 0, 0}); // move
    // four.push_back(SwerveDrivePathGenerator::waypoint_t {54, 249, 130, (RobotParameters::k_maxSpeed)*39.38, 0}); // other end
    // four.push_back(SwerveDrivePathGenerator::waypoint_t {77, 240, 189, 0, 0}); // other end
    //101.67+2.39-2.868-23.99-.877-.877, 222.756
    std::vector<SwerveDrivePathGenerator::waypoint_t> tempWaypoints;//if meters
    

    AddCommands(
        PathFollowerShootingCommand(m_drive, m_shooter, m_feeder, 1, one, "start path" ,true),
        PathFollowerCommand(m_drive, two, "first path")
      // frc2::ParallelRaceGroup{
      //   FeederDefaultCommand(m_feeder),
      //   frc2::SequentialCommandGroup{
      //     // TurnLimeLightOff(),
      //     ExtendIntakeCommand(m_intake),
      //     PathFollowerCommand(m_drive, start, "start path" ,true),//head to end
      //     frc2::ParallelCommandGroup{
      //       PathFollowerCommand(m_drive, first, "first path"),
      //       RetractIntakeCommand(m_intake)
      //     },
      //     StartShooterCommand(m_shooter),
      //     frc2::ParallelCommandGroup{
      //       SetShooterSpeedCommand(m_shooter,ShooterConstants::kDefaultShooterShortSpeed),
      //       RotateWithMotionMagic(m_drive, 97,1, true)
      //       // TurnLimeLightOn(),
      //       // LimeLightRotateTillOnTarget(m_drive, 97, 2)
            
      //     }
      //   }
      // },
      
      // frc2::ParallelRaceGroup{
      //   ShootBallCommand(m_shooter, m_feeder),
      //   frc2::WaitCommand(2.5_s)//5
      // },
      
      // frc2::ParallelRaceGroup{
      //   FeederDefaultCommand(m_feeder),
      //   frc2::SequentialCommandGroup{
      //     frc2::ParallelCommandGroup{
      //       StopShooterCommand(m_shooter),
      //       // TurnLimeLightOff(),
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
      //     // TurnLimeLightOn(),
      //     // LimeLightRotateTillOnTarget(m_drive, 97, 2)
      //   },
      // frc2::ParallelRaceGroup{
      //   ShootBallCommand(m_shooter, m_feeder),
      //   frc2::WaitCommand(3_s)
      // },
      
      // StopShooterCommand(m_shooter)
      //     // TurnLimeLightOff()
    );
  }
};
