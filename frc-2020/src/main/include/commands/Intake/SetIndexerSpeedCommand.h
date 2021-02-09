/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/FeederSubsystem.h"
class SetIndexerSpeedCommand
    : public frc2::CommandHelper<frc2::CommandBase,
                                 SetIndexerSpeedCommand> {
 private:
  IndexerSubsystem* m_indexer;
  FeederSubsystem* m_feeder;
  double m_speed;

 public:
  SetIndexerSpeedCommand(IndexerSubsystem* indexer, FeederSubsystem* feeder, double speed){
    m_indexer = indexer;
    m_feeder = feeder;
    m_speed = speed;
    
    AddRequirements(m_feeder);
    AddRequirements(m_indexer);

  }

  void Initialize() override{
    // printf("SetFeederSpeedCommand\n");
    m_indexer->setIndexerSpeed(m_speed);
    m_feeder->setFeederSpeed(1);
  }
  void Execute() override{}
  void End(bool interrupted) override{
    m_indexer->setIndexerSpeed(0);
    m_feeder->setFeederSpeed(0);
  }

  bool IsFinished() override{
    return false;
  }
};