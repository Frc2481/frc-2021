/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include "Utils/SwerveDrivePathFollower.h"
#include "subsystems/DriveSubsystem.h"

class StopPathFollowing
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 StopPathFollowing> {
 private:
  SwerveDrivePathFollower* m_pFollower;
  DriveSubsystem* m_pDriveSubsystem;
 public:
  StopPathFollowing(SwerveDrivePathFollower* follower, DriveSubsystem* driveSubsystem){
    m_pFollower = follower;
    m_pDriveSubsystem = driveSubsystem;
    AddRequirements(m_pDriveSubsystem);
  }

  void Initialize() override{
    m_pDriveSubsystem->stop();
    m_pFollower->stop(false);
  }
};
