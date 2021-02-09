/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include "subsystems/ShooterSubsystem.h"
#include "Constants.h"
class SetCloseShotSpeedCommand
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 SetCloseShotSpeedCommand> {
 private:
 ShooterSubsystem* m_pShooter;
 public:
  SetCloseShotSpeedCommand(ShooterSubsystem* Shooter){
    m_pShooter = Shooter;
    AddRequirements(m_pShooter);
  }

  void Initialize() override{
    m_pShooter->setShooterSpeed(ShooterConstants::kDefaultShooterShortSpeed);
  }
};
