/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/IndexerSubsystem.h"
#include "subsystems/FeederSubsystem.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class IndexerDefaultCommand
    : public frc2::CommandHelper<frc2::CommandBase, IndexerDefaultCommand> {
 private:
  IndexerSubsystem* m_pIndexer;
  FeederSubsystem* m_pFeeder;
 public:
  IndexerDefaultCommand(IndexerSubsystem* indexer, FeederSubsystem* feeder){
    m_pFeeder = feeder;
    m_pIndexer = indexer;
    AddRequirements(m_pIndexer);
  }

  void Initialize() override{}

  void Execute() override{
    if((m_pIndexer->isBallInTrench() && !m_pIndexer->isBallInBottomBeamBreak()) //we have a ball to move to the bottom beam brake
            || (m_pFeeder->isFeederReady() && (m_pIndexer->isBallInBottomBeamBreak()))){//we have ball to give feeder and the feeder is read)y
      m_pIndexer->setIndexerSpeed(IndexerConstants::kDefaultIntakeIndexerSpeed);
    }else{
      m_pIndexer->setIndexerSpeed(0);
    }
  }
  void End(bool interrupted) override{
    m_pIndexer->setIndexerSpeed(0);
  }

  bool IsFinished() override{
    return false;
  }
};
