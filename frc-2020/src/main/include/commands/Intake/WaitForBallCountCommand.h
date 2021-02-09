/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/FeederSubsystem.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class WaitForBallCountCommand
    : public frc2::CommandHelper<frc2::CommandBase, WaitForBallCountCommand> {
 private:
  FeederSubsystem* m_pFeeder;
  int m_ballWanted;
 public:
  WaitForBallCountCommand(FeederSubsystem* feeder, int ballWanted){
    m_pFeeder = feeder;
    m_ballWanted = ballWanted;
  }

  void Initialize() override{}

  void Execute() override{}

  void End(bool interrupted) override{}

  bool IsFinished() override{
    return m_pFeeder->getBallCount() >= m_ballWanted;
  }
};
