/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/InstantCommand.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ClimberSubsystem.h"
#include "Constants.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ClimberPullUpCommand
    : public frc2::CommandHelper<frc2::InstantCommand, ClimberPullUpCommand> {
 private:
 ClimberSubsystem* m_pClimber;
 public:
  ClimberPullUpCommand(ClimberSubsystem* climber){
    m_pClimber = climber;
    AddRequirements(m_pClimber);
  }

  void Initialize() override{
    m_pClimber->goToPos(ClimberConstants::kClimbingZero);
  }
};
