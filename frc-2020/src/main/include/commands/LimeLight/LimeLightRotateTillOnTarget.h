/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "subsystems/DriveSubsystem.h"
#include "commands/Drive/RotateWithMotionMagic.h"
#include <frc2/command/ParallelRaceGroup.h>
#include "commands/LimeLight/LimeLightVisable.h"
class LimeLightRotateTillOnTarget
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 LimeLightRotateTillOnTarget> {
 public:
  LimeLightRotateTillOnTarget(DriveSubsystem* drive, double targetAngle, double range){
    AddCommands(
      frc2::ParallelRaceGroup{
        RotateWithMotionMagic(drive, targetAngle,range),
        LimeLightVisable()
      },
      RotateWithMotionMagic(drive, targetAngle,range, true)
    );
  }
};
