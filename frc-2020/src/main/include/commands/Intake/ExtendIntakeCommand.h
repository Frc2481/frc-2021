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
#include "Constants.h"
class ExtendIntakeCommand
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 ExtendIntakeCommand> {
 private:
  IntakeSubsystem* m_intake;

 public:
  ExtendIntakeCommand(IntakeSubsystem* intake){
    m_intake = intake;
    AddRequirements(m_intake);
  }

  void Initialize() override{
    m_intake->extend();
    m_intake->setRollerSpeed(IntakeConstants::kDefaultIntakeRollerSpeed);
  }
  
};