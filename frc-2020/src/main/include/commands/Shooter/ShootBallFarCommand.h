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

class ShootBallFarCommand
    : public frc2::CommandHelper<frc2::CommandBase, ShootBallFarCommand> {

 private:
  ShooterSubsystem* m_shooter;
  FeederSubsystem* m_feeder;
  bool m_finished;
  
 public:
  ShootBallFarCommand(ShooterSubsystem* shooter, FeederSubsystem* feeder) {
    m_shooter = shooter;
    m_feeder = feeder;
    m_finished = false;
    AddRequirements(m_shooter);
    AddRequirements(m_feeder);
    
  }

  void Initialize() override{
    // printf("ShootBallFarCommand\n");
    // if(m_shooter->distanceToTarget() <= ShooterConstants::kCloseRangeDistance && m_shooter->distanceToTarget() != -1){
    //   m_shooter->setCloseShot(true);
    // }else{//TODO work on
    //   m_shooter->setCloseShot(false);
    // }
    if(!m_shooter->isShooterOn()){
      m_finished = true;
    }
  }

  void Execute() override{
    if(m_shooter->isShooterOnTarget()){
      m_feeder->setFeederSpeed(FeederConstants::kShootingFarFeederSpeed);
      m_finished = false;
    }
  }

  bool IsFinished() override{
    return m_finished;
  }

  void End(bool interrupted) override{
    // frc::SmartDashboard::PutBoolean("feeder interupted", interrupted);
    m_feeder->setFeederSpeed(0.0);
  }
  
};
