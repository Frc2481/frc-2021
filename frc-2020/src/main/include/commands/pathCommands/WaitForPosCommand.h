/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveSubsystem.h"
#include <frc/geometry/Translation2d.h>
#include <units/units.h>
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class WaitForPosCommand
    : public frc2::CommandHelper<frc2::CommandBase, WaitForPosCommand> {
 private:
  DriveSubsystem *m_pDriveSubsystem;
  double m_xThresh;
  double m_yThresh;
  double m_xPos;
  double m_yPos;
 public:
  WaitForPosCommand(DriveSubsystem* driveSubsystem, double xPos, double yPos, double xThresh, double yThresh){
    m_xThresh = xThresh;
    m_yThresh = yThresh;
    m_pDriveSubsystem = driveSubsystem;
    // AddRequirements(m_pDriveSubsystem);
    m_xPos = xPos;
    m_yPos = yPos;
  }

  void Initialize() override{}

  void Execute() override{}

  void End(bool interrupted) override{}

  bool IsFinished() override{
    return fabs(m_xPos - m_pDriveSubsystem->GetPose().Translation().X().to<double>()) < m_xThresh && 
           fabs(m_yPos - m_pDriveSubsystem->GetPose().Translation().Y().to<double>()) <m_yThresh;
  }
};
