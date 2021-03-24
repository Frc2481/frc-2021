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
#include "commands/Drive/EngageCoastCommand.h"
class AutoNavPathB
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AutoNavPathB> {
 private:
 public:
  AutoNavPathB(DriveSubsystem* m_drive, SwerveDrivePathFollower* m_follower){
   
    AddCommands(
      //Break Into Multiple Commands
      EngageCoastCommand(m_drive),
        PathFollowerCommand2(m_drive, m_follower, "path path" ,true),
        EngageBakeCommand(m_drive)
    );
  }
};
