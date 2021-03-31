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
#include "subsystems/IntakeSubsystem.h"

#include "commands/Intake/IntakesDefaultCommand.h"
#include "commands/Intake/DropIntake.h"
#include "commands/Drive/EngageBakeCommand.h"
#include "commands/Drive/EngageBakeCommand.h"
class GalacticSearchPathARed
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 GalacticSearchPathARed> {
 private:
  
 public:
  GalacticSearchPathARed(DriveSubsystem* m_drive,
                      IntakeSubsystem* m_intake,
                      SwerveDrivePathFollower* m_follower){

    
    AddCommands(
      frc2::ParallelRaceGroup{
        // DropIntake(m_intake),
        IntakesDefaultCommand(m_intake),

        PathFollowerCommand2(m_drive, m_follower, "path path" ,true),
        
      },
      EngageBakeCommand(m_drive)
    );
  }
};
