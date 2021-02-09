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

class StartShooterCommand
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 StartShooterCommand> {
 private:
  ShooterSubsystem* m_shooter;

 public:
  StartShooterCommand(ShooterSubsystem* shooter){
    m_shooter = shooter;
    AddRequirements(m_shooter);
  }

  void Initialize() override{
    // printf("shooterStarted\n");
    m_shooter->startShooter();
  }
};
