/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"
#include <frc/Timer.h>
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RetractIntakeCommand
    : public frc2::CommandHelper<frc2::CommandBase, RetractIntakeCommand> {
 private:
  IntakeSubsystem* m_pIntake;
  frc::Timer m_timer;
 public:
  RetractIntakeCommand(IntakeSubsystem* intake){
    m_pIntake = intake;
    
    AddRequirements(m_pIntake);
  }

  void Initialize() override{
    m_timer.Start();
    m_pIntake->retract();
  }
  void Execute() override{}
  void End(bool interrupted) override{
    m_pIntake->setRollerSpeed(0);
    m_timer.Stop();
    m_timer.Reset();
  }

  bool IsFinished() override{
    return m_timer.Get() >= .5;
  }
};
