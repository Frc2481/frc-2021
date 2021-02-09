/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include "subsystems/ClimberSubsystem.h"
class ZeroClimberCommand
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 ZeroClimberCommand> {
 private:
  ClimberSubsystem* m_pClimber;
 public:
  ZeroClimberCommand(ClimberSubsystem* climber){
    m_pClimber = climber;
    AddRequirements(m_pClimber);
  }

  void Initialize() override{
    m_pClimber->zeroPos();
  }
};
