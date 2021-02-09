/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once


#include <frc2/command/InstantCommand.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ShooterSubsystem.h"

class SetShooterSpeedCommand
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 SetShooterSpeedCommand> {
 private:
  ShooterSubsystem* m_shooter;
  double m_speed;
 public:
  SetShooterSpeedCommand(ShooterSubsystem* shooter, double speed){
    m_shooter = shooter;
    m_speed = speed;
    
    AddRequirements(m_shooter);
  }

  void Initialize() override{
    m_shooter->startShooter();
    if(m_speed <= 0){
      m_shooter->setShooterSpeed(ShooterConstants::kDefaultShooterFarSpeed);
      
    }
    m_shooter->setShooterSpeed(m_speed);
    // m_shooter->setShooterSpeed(m_speed  < 0? frc::SmartDashboard::GetNumber("shooter Set Point", ShooterConstants::): m_speed);
  }

};
