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
#include "Constants.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class FeederDefaultCommand
    : public frc2::CommandHelper<frc2::CommandBase, FeederDefaultCommand> {
 private:
  FeederSubsystem *m_pFeeder;
  bool m_prev = false;
 public:
  FeederDefaultCommand(FeederSubsystem *feeder){
    m_pFeeder = feeder;
    
    AddRequirements(m_pFeeder);
  }

  void Initialize() override{
    // printf("feeder started\n");
  }

  void Execute() override{
    if(m_pFeeder->isBallInMidBeamBreak()  &&!m_pFeeder->isBallInTopBeamBreak()){
      // printf("you are setting feeder pos\n");
      m_pFeeder->setFeederPos(FeederConstants::kIntakeBallDistance);//TODO get off dashboard
      // printf("you have set feeder pos\n");
    }else if(m_pFeeder->isBallInTopBeamBreak()){
      m_pFeeder->setFeederSpeed(0);
    }
  }

  void End(bool interrupted) override{
    m_pFeeder->setFeederSpeed(0);
    // printf("feeder interrupted %d\n", interrupted);
  }

  bool IsFinished() override{
    return false;
  }
};
