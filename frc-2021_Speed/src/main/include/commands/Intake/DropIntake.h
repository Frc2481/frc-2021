// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>

#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"

class DropIntake
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 DropIntake> {
 private:
 IntakeSubsystem* m_pIntake;
 public:
  DropIntake(IntakeSubsystem* intake){
    m_pIntake = intake;
  }

  void Initialize() override{
    m_pIntake->setServoAngle(0);//180
  }
};
