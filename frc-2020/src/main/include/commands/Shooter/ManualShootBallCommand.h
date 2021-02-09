/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/FeederSubsystem.h"
#include <frc/SmartDashboard/smartDashboard.h>
#include "subsystems/IndexerSubsystem.h"
class ManualShootBallCommand
    : public frc2::CommandHelper<frc2::CommandBase, ManualShootBallCommand> {

 private:
  ShooterSubsystem* m_shooter;
  FeederSubsystem* m_feeder;
  IndexerSubsystem* m_indexer;
  
 public:
  ManualShootBallCommand(ShooterSubsystem* shooter, FeederSubsystem* feeder, IndexerSubsystem* indexer) {
    m_shooter = shooter;
    m_feeder = feeder;
    m_indexer = indexer;
    AddRequirements(m_indexer);
    AddRequirements(m_feeder);
    AddRequirements(m_shooter);
    
  }

  void Initialize() override{
    // printf("ManualShootBallCommand\n");
    m_feeder->setFeederSpeed(FeederConstants::kShootingFeederSpeed);
    m_indexer->setIndexerSpeed(IndexerConstants::kDefaultIntakeIndexerSpeed);
  }

  void Execute() override{
  }

  bool IsFinished() override{
    return false;
  }

  void End(bool interrupted) override{
    m_feeder->setFeederSpeed(0);
    m_indexer->setIndexerSpeed(0);
  }
  
};
