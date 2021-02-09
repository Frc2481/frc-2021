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
#include "subsystems/IndexerSubsystem.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ManualIntakeBallsCommand
    : public frc2::CommandHelper<frc2::CommandBase, ManualIntakeBallsCommand> {
 private:
 IndexerSubsystem* m_indexer;
  IntakeSubsystem* m_intake;
  FeederSubsystem* m_feeder;
  bool m_fromGround;
  bool m_prevState = false;
 public:
  ManualIntakeBallsCommand(IntakeSubsystem* intake, IndexerSubsystem* indexer, FeederSubsystem* feeder, bool fromGround){
    m_intake = intake;
    m_feeder = feeder;
    m_indexer = indexer;
    m_fromGround = fromGround;
    AddRequirements(m_intake);
    AddRequirements(m_feeder);
    AddRequirements(m_indexer);
    
  }

  void Initialize() override{
    // printf("ManualIntakeBallsCommand\n");
    if(m_fromGround){
      m_intake->setRollerSpeed(-IntakeConstants::kDefaultIntakeRollerSpeed);
      m_intake->extend();
    }
    m_indexer->setIndexerSpeed(IndexerConstants::kDefaultIntakeIndexerSpeed);
    
    
  }

  void Execute() override{
   
  }

  void End(bool interrupted) override{
    if(m_fromGround){
      m_intake->setRollerSpeed(0.0);
      m_intake->retract();
    }
    m_indexer->setIndexerSpeed(0);
    m_feeder->setFeederSpeed(0);
  }

  bool IsFinished() override{
    return false;
  }
};
