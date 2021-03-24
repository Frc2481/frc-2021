// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "commands/Autos/GalacticSearchPathABlue.h"
#include "commands/Autos/GalacticSearchPathBBlue.h"
#include "commands/Autos/GalacticSearchPathARed.h"
#include "commands/Autos/GalacticSearchPathBRed.h"
#include "networktables/NetworkTableInstance.h"

#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "Utils/SwerveDrivePathFollower.h"

using namespace nt;
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoSelectorCommandPath
    : public frc2::CommandHelper<frc2::CommandBase, AutoSelectorCommandPath> {
 private:
 DriveSubsystem* m_pDrive;
 IntakeSubsystem* m_pIntake;
 SwerveDrivePathFollower* m_followerABlue;
 SwerveDrivePathFollower* m_followerBBlue;
 SwerveDrivePathFollower* m_followerARed;
 SwerveDrivePathFollower* m_followerBRed;
 double targetDistance = 0;
 double targetY = 0;
 bool m_finished = false;
 public:
  AutoSelectorCommandPath(DriveSubsystem* drive, IntakeSubsystem* intake, SwerveDrivePathFollower* followerABlue, SwerveDrivePathFollower* followerBBlue, SwerveDrivePathFollower* followerARed, SwerveDrivePathFollower* followerBRed){
    m_pDrive = drive;
    m_pIntake = intake;
    m_followerABlue = followerABlue;
    m_followerBBlue = followerBBlue;
    m_followerARed = followerARed;
    m_followerBRed = followerBRed;
    AddRequirements(m_pDrive);
    AddRequirements(m_pIntake);
  }

  void Initialize() override{
    
  }

  void Execute() override{
    if((bool)NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv",0)){
      targetDistance = NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0);
      targetY = NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0);
      if(targetDistance > 30 && targetDistance < 120){//RED
        if(targetY > 0){
          GalacticSearchPathARed(m_pDrive, m_pIntake, m_followerARed);
          m_finished = true;
        }else{
          GalacticSearchPathBRed(m_pDrive, m_pIntake, m_followerBRed);
          m_finished = true;
        }
      }else if(targetDistance > 120 && targetDistance < 300){//Blue
        if(targetY > 0){
          GalacticSearchPathABlue(m_pDrive, m_pIntake, m_followerABlue);
          m_finished = true;
        }else{
          GalacticSearchPathBBlue(m_pDrive, m_pIntake, m_followerBBlue);
          m_finished = true;
        }
      }
    }
  }

  void End(bool interrupted) override{

  }

  bool IsFinished() override{
    return m_finished;
  }
};
