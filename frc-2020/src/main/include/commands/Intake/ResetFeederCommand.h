/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include "subsystems/FeederSubsystem.h"
class ResetFeederCommand
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 ResetFeederCommand> {
 private:
  FeederSubsystem* m_pFeeder;
 public:
  ResetFeederCommand(FeederSubsystem* feeder){
    
    m_pFeeder = feeder;
  }

  void Initialize() override{
    m_pFeeder->setFeederPos(29);
  }
};
