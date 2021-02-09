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
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ClimberToPos
    : public frc2::CommandHelper<frc2::InstantCommand, ClimberToPos> {
 ClimberSubsystem* m_pClimber;
 double m_pos;
 public:
  ClimberToPos(ClimberSubsystem* climber, double temp){
    m_pClimber = climber;
    AddRequirements(m_pClimber);
  }

  void Initialize() override{
    m_pClimber->goToPos(frc::SmartDashboard::GetNumber("climberSetPos", 0));
  }
};
