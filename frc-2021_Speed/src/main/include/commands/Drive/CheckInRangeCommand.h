// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class CheckInRangeCommand
    : public frc2::CommandHelper<frc2::CommandBase, CheckInRangeCommand> {
      private: 
      DriveSubsystem* m_pDriveSubsystem;
      double m_xRange;
      double m_xStart = 0.0;
      bool m_isGreaterThanRange;
      int m_greaterThanOveride;
 public:

  //drive subsystem, point on x axis greaterThanOveride 0: less than, 1: greater than, 2: false
  CheckInRangeCommand(DriveSubsystem* driveSubsystem, double xRange, int greaterThanOveride = 2){
  m_pDriveSubsystem = driveSubsystem;
  m_xRange = xRange;
  m_greaterThanOveride = greaterThanOveride;
  }

  void Initialize() override{
    m_xStart = m_pDriveSubsystem->GetPose().Translation().X().to<double>();
    if(m_greaterThanOveride == 2){
      m_isGreaterThanRange = m_xStart < m_xRange;
    }else{
      m_isGreaterThanRange = (bool)m_greaterThanOveride;
    }
  }

  void Execute() override{

  }

  void End(bool interrupted) override{
    printf("You are in range!");
  }

  bool IsFinished() override{
    return m_isGreaterThanRange ? m_pDriveSubsystem->GetPose().Translation().X().to<double>() > m_xRange : m_pDriveSubsystem->GetPose().Translation().X().to<double>() < m_xRange;
  }
};
