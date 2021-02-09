/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/InstantCommand.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ShooterSubsystem.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SetShooterRangeCommand
    : public frc2::CommandHelper<frc2::InstantCommand, SetShooterRangeCommand> {
  private:
    ShooterSubsystem* m_pShooter;
    bool m_close;
 public:
  SetShooterRangeCommand(ShooterSubsystem* shooter, bool close){
    m_close = close;
    m_pShooter = shooter;
    AddRequirements(m_pShooter);
  }

  void Initialize() override{
    m_pShooter->setCloseShot(m_close);
  }
};
