/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include "subsystems/ShooterSubsystem.h"

class StopShooterCommand
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 StopShooterCommand> {
 private:
  ShooterSubsystem* m_shooter;

 public:
  StopShooterCommand(ShooterSubsystem* shooter){
    m_shooter = shooter;
    AddRequirements(m_shooter);

  }

  void Initialize() override{
    // printf("shooterStopped");
    m_shooter->stopShooter();
  }
};