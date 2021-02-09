/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ShooterSubsystem.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ToggleDinkerCommand
    : public frc2::CommandHelper<frc2::CommandBase, ToggleDinkerCommand> {
 ShooterSubsystem* m_pShooter;
 public:
  ToggleDinkerCommand(ShooterSubsystem* shooter){
    m_pShooter = shooter;
  }

  void Initialize() override{
    m_pShooter->setCloseShot(true);
  }

  void Execute() override{}

  void End(bool interrupted) override{
    m_pShooter->setCloseShot(false);
  }

  bool IsFinished() override{
    return false;
  }
};
