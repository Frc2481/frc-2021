// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include "subsystems/IntakeSubsystem.h"
class SetRightIntakeSpeed
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 SetRightIntakeSpeed> {
 private:
IntakeSubsystem* m_pIntake;
double m_speed;
 public:
  SetRightIntakeSpeed(IntakeSubsystem* intake, double speed){
    m_pIntake = intake;
    m_speed = speed;
    AddRequirements(m_pIntake);
  }

  void Initialize() override{
    m_pIntake->setLeftSpeed(m_speed);//TODO check if inverted
  }
};
