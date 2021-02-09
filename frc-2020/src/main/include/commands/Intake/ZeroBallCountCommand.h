/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/InstantCommand.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/FeederSubsystem.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ZeroBallCountCommand
    : public frc2::CommandHelper<frc2::InstantCommand, ZeroBallCountCommand> {
 private:
 FeederSubsystem *m_pFeeder;
 public:
  ZeroBallCountCommand(FeederSubsystem* feeder){
    m_pFeeder = feeder;
  }

  void Initialize() override{
    // printf("ZeroBallCountCommand\n");
    m_pFeeder->zeroBallCount();
  }
};
