/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <vector>
#include <limits>
#include <algorithm>
#include "Utils/SwerveDrivePathGenerator.h"
#include "subsystems/DriveSubsystem.h"
#include <frc/geometry/Translation2d.h>
#include <units/velocity.h>
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class WaitForPathToFinishCommand
    : public frc2::CommandHelper<frc2::CommandBase, WaitForPathToFinishCommand> {
 private:
  std::vector<SwerveDrivePathGenerator::finalPathPoint_t> m_path;
  DriveSubsystem *m_pDriveTrain;
  int m_targetRange;
 public:
  WaitForPathToFinishCommand(std::vector<SwerveDrivePathGenerator::finalPathPoint_t> &path, DriveSubsystem* driveTrain, int targetRange){
    m_path = path;
    m_targetRange = targetRange;
    m_pDriveTrain = driveTrain;
  }

  void Initialize() override{}

  void End(bool interrupted) override{}

  bool IsFinished() override{
    return (m_pDriveTrain->GetPose().Translation() - 
            frc::Translation2d(units::meter_t(m_path[m_path.size()-1].xPos), 
                               units::meter_t(m_path[m_path.size()-1].yPos))
            ).Norm().to<double>() < m_targetRange;
  }
};
