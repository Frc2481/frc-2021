// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
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

class IntakesDefaultCommand
    : public frc2::CommandHelper<frc2::CommandBase, IntakesDefaultCommand> {
  private:
     IntakeSubsystem* m_pIntake;
     int count;
 public:
  IntakesDefaultCommand(IntakeSubsystem* pIntake){
    m_pIntake = pIntake;
    AddRequirements(m_pIntake);
  }

  void Initialize() override{
    m_pIntake->setAIntakeSpeed(IntakeConstants::kAIntakeSpeed);
    m_pIntake->setBIntakeSpeed(IntakeConstants::kBIntakeSpeed);
  }

  void Execute() override{
    if(m_pIntake->getIntakeACurrent() >= IntakeConstants::kMaxIntakeAmp){
      count++;
    }else{
      count = 0;
    }
    if(count > 7){
      m_pIntake->setAIntakeSpeed(0);
    }
  }

  void End(bool interrupted) override{
    m_pIntake->setAIntakeSpeed(0);
    m_pIntake->setBIntakeSpeed(0);
    printf("intake was Interupted: %d", (int)interrupted);
  }

  bool IsFinished() override{
    return false;
  }
};
