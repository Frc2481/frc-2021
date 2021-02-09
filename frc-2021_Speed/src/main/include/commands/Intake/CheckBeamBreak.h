// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include "subsystems/IntakeSubsystem.h"
class CheckBeamBreak
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 CheckBeamBreak> {
 private:
 IntakeSubsystem* m_pIntake;
 public:
  CheckBeamBreak(IntakeSubsystem* intake){
    m_pIntake = intake;
    AddRequirements(m_pIntake);
  }

  void Initialize() override{
    if(m_pIntake->getBeamBreak()){
      m_pIntake->setLeftSpeed(0);//TODO check if it is right or left
    }
  }

};
