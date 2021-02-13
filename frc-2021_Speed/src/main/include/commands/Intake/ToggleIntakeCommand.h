// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/InstantCommand.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/IntakeSubsystem.h"

#include "Constants.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ToggleIntakeCommand
    : public frc2::CommandHelper<frc2::InstantCommand, ToggleIntakeCommand> {
      private:
      IntakeSubsystem* m_pIntake;
 public:
  ToggleIntakeCommand(IntakeSubsystem* pIntake){
    m_pIntake = pIntake;
  }

  void Initialize() override{
    if(m_pIntake->getRightSpeed() != 0){
      m_pIntake->setRightSpeed(0);
      m_pIntake->setLeftSpeed(0);
    }
    else{
      m_pIntake->setRightSpeed(IntakeConstants::kRightIntakeSpeed);
      m_pIntake->setLeftSpeed(IntakeConstants::kLeftIntakeSpeed);
    }
  }
};
