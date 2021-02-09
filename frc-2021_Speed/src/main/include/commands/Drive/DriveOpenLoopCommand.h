/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/InstantCommand.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveSubsystem.h"
#include <units/units.h>
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveOpenLoopCommand
    : public frc2::CommandHelper<frc2::InstantCommand, DriveOpenLoopCommand> {
 private:
  DriveSubsystem* m_pDriveSubsystem;
  units::meters_per_second_t m_x;
  units::meters_per_second_t m_y;
  units::radians_per_second_t m_yawRate;
  bool m_fieldCentric;

 public:
 //DriveSubsystem, x in meters per second, y meters per second, yawRate degrees per second
  DriveOpenLoopCommand(DriveSubsystem* driveSubsystem, 
                       units::meters_per_second_t x, 
                       units::meters_per_second_t y, 
                       units::radians_per_second_t yawRate, 
                       bool fieldCentric){
    m_pDriveSubsystem = driveSubsystem;
    m_x = x;
    m_y = y;
    m_yawRate = yawRate;
    m_fieldCentric = fieldCentric;
    AddRequirements(m_pDriveSubsystem);
  }

  void Initialize() override{
    m_pDriveSubsystem->Drive(m_y, 
                             m_x, 
                             m_yawRate, 
                             m_fieldCentric);
  }
};
