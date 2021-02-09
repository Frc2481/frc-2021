/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include "subsystems/IntakeSubsystem.h"

class SetRollerSpeedCommand
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 SetRollerSpeedCommand> {
 private:
  IntakeSubsystem* m_roller;

 public:
  SetRollerSpeedCommand(IntakeSubsystem *roller){
    m_roller = roller;
    AddRequirements(m_roller);

  }

  void Initialize() override{
    if(fabs(m_roller->getRollerSpeed()) > 0){
      m_roller->setRollerSpeed(0);
    }
    else{
      m_roller->setRollerSpeed(.1);
    }
  }
};
