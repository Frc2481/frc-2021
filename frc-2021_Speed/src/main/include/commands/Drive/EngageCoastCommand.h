// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>

#include "subsystems/DriveSubsystem.h"

class EngageCoastCommand
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 EngageCoastCommand> {
 private:
  DriveSubsystem* m_pDrive;
 public:
  EngageCoastCommand(DriveSubsystem* drive){
    m_pDrive = drive;
  }

  void Initialize() override{
    m_pDrive->setCoast();
  }
};
