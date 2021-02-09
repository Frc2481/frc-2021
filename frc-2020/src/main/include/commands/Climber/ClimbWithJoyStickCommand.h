/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "components/Joystick2481.h"
#include "subsystems/ClimberSubsystem.h"
#include "components/XboxController2481.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ClimbWithJoyStickCommand
    : public frc2::CommandHelper<frc2::CommandBase, ClimbWithJoyStickCommand> {
  private:
    Joystick2481* m_pAux;
    ClimberSubsystem* m_pClimber;
 public:
  ClimbWithJoyStickCommand(ClimberSubsystem* climber, Joystick2481* aux){
    m_pAux = aux;
    m_pClimber = climber;
    AddRequirements(m_pClimber);
  }

  void Initialize() override{

  }

  void Execute() override{
    if(m_pClimber->climberReady()){
      m_pClimber->climb(-m_pAux->GetRawAxis(XBOX_LEFT_Y_AXIS));
    }
    
  }

  void End(bool interrupted) override{
    m_pClimber->climb(0);
  }

  bool IsFinished() override{
    return false;
  }
};
