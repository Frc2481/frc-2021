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
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SetFeederSpeedCommand
    : public frc2::CommandHelper<frc2::CommandBase, SetFeederSpeedCommand> {
 private:
  FeederSubsystem*  m_pFeeder;
  double m_speed;
 public:
  SetFeederSpeedCommand(FeederSubsystem* feeder, double speed){
    m_pFeeder = feeder;
    m_speed = speed;

    AddRequirements(m_pFeeder);
  }

  void Initialize() override{
        // printf("SetFeederSpeedCommand\n");
    m_pFeeder->setFeederSpeed(m_speed);
  }

  void Execute() override{
    
  }

  void End(bool interrupted) override{
    m_pFeeder->setFeederSpeed(0);
  }

  bool IsFinished() override{
    return false;
  }
};
