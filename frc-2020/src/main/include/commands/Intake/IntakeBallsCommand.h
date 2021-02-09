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
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/FeederSubsystem.h"
#include "Constants.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class IntakeBallsCommand
    : public frc2::CommandHelper<frc2::CommandBase, IntakeBallsCommand> {
 private:
  IntakeSubsystem* m_intake;
  FeederSubsystem* m_feeder;
  bool m_fromGround;
  bool m_prevState = false;
 public:
  IntakeBallsCommand(IntakeSubsystem* intake, FeederSubsystem* feeder, bool fromGround, bool overide = false){
    m_intake = intake;
    m_feeder = feeder;
    m_fromGround = fromGround;
    AddRequirements(m_intake);
    AddRequirements(m_feeder);
  }

  void Initialize() override{
    
    // printf("IntakeBallsCommand\n");
    if(m_fromGround){
      m_intake->setRollerSpeed(-IntakeConstants::kDefaultIntakeRollerSpeed);
      m_intake->extend();
    }
    m_intake->setIndexerSpeed(IntakeConstants::kDefaultIntakeIndexerSpeed);
    
    
  }

  void Execute() override{
    // printf("-----------intaking--------------\n");
    if(((m_feeder->getBallCount() < 5) && !m_feeder->isBallInTopBeamBreak())){ 
      if((m_feeder->isBallInBeamBreak() || m_feeder->isBallInMidBeamBreak()) && !m_prevState){
        m_feeder->zeroPos();
        m_feeder->setFeederPos(frc::SmartDashboard::GetNumber("ball distance", FeederConstants::kIntakeBallDistance));
      }
      // if(m_feeder->isBallInBeamBreak() || m_feeder->isBallInMidBeamBreak()){
      //   m_feeder->setFeederSpeed(1);
      // }else{
      //   m_feeder->setFeederSpeed(0);
      // }
    }else if(m_feeder->isBallInExtraBeamBreak()){
      m_feeder->setFeederSpeed(0);
      m_intake->setRollerSpeed(0.0);
      m_intake->setIndexerSpeed(0.0);
      m_intake->retract();
    }else{
      m_feeder->setFeederSpeed(0);
    }


    if((m_feeder->isBallInBeamBreak() || m_feeder->isBallInMidBeamBreak())){
        m_prevState = true;
    }else if(!(m_feeder->isBallInBeamBreak() || m_feeder->isBallInMidBeamBreak())){
      m_prevState = false;
    }
  }

  void End(bool interrupted) override{
    if(m_fromGround){
      m_intake->setRollerSpeed(0.0);
      m_intake->retract();
    }
    m_intake->setIndexerSpeed(0);
    m_feeder->setFeederSpeed(0);
  }

  bool IsFinished() override{
    return false;
  }
};
