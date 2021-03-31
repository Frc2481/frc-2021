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
class DropBallsCommand
    : public frc2::CommandHelper<frc2::CommandBase, DropBallsCommand> {
 private:
  IntakeSubsystem* m_pIntake;
 public:
  DropBallsCommand(IntakeSubsystem* intake){
    m_pIntake = intake;
    AddRequirements(m_pIntake);
  }

  void Initialize() override{
    m_pIntake->setAIntakeSpeed(IntakeConstants::kAIntakeReverse);
    m_pIntake->setBIntakeSpeed(IntakeConstants::kBIntakeReverse);
  }

  void Execute() override{
    m_pIntake->getAIntakeSpeed();
  }

  void End(bool interrupted) override{
    m_pIntake->setAIntakeSpeed(0);
    m_pIntake->setBIntakeSpeed(0);
  }

  bool IsFinished() override{
    return false;
  }
};
