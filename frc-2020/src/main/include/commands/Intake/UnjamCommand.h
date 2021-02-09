/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "Subsystems/FeederSubsystem.h"
#include "subsystems/IndexerSubsystem.h"
#include "Constants.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class UnjamCommand
    : public frc2::CommandHelper<frc2::CommandBase, UnjamCommand> {
 private:
  FeederSubsystem* m_pFeeder;
  IndexerSubsystem* m_pIndexer;
 public:
  UnjamCommand(IndexerSubsystem* indexer, FeederSubsystem* feeder){
    m_pFeeder = feeder;
    m_pIndexer = indexer;
    AddRequirements(m_pFeeder);
    AddRequirements(m_pIndexer);
   
  }

  void Initialize() override{
    //  printf("UnjamCommand\n");
  }

  void Execute() override{
    m_pFeeder->setFeederSpeed(-.5);
    m_pIndexer->setIndexerSpeed(.2);
  }

  void End(bool interrupted) override{
    m_pFeeder->setFeederSpeed(0);
    m_pIndexer->setIndexerSpeed(0);
    m_pFeeder->setSensorPos(FeederConstants::kIntakeBallDistance +1);
  }

  bool IsFinished() override{
    return false;
  }
};
