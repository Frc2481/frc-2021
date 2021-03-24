// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/IntakeSubsystem.h"
#include <frc/Timer.h>
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
     int countB;
     int countA;
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
      countA++;
    }else{
      countA = 0;
    }
    if(countA > 7){
      m_pIntake->setAIntakeSpeed(0);
      frc::Wait(.05);
      m_pIntake->setAIntakeSpeed(IntakeConstants::kBIntakeSpeed);
    }
    if(m_pIntake->getIntakeBCurrent() >= IntakeConstants::kMaxIntakeAmp){
      countB++;
    }else{
      countB = 0;
    }
    if(countB > 3){
      m_pIntake->setBIntakeSpeed(0);
      frc::Wait(.05);
      m_pIntake->setBIntakeSpeed(IntakeConstants::kBIntakeSpeed);
    }
  }

  void End(bool interrupted) override{
    m_pIntake->setAIntakeSpeed(0);
    m_pIntake->setBIntakeSpeed(0);
    countA = 0;
    countB = 0;
    printf("intake was Interupted: %d", (int)interrupted);
  }

  bool IsFinished() override{
    return false;
  }
};
