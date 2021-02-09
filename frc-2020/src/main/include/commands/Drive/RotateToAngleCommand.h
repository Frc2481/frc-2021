
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "RobotContainer.h"
#include "subsystems/DriveSubsystem.h"
#include <wpi/math>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/ProfiledPIDController.h>
#include "RobotParameters.h"
#include "Utils/NormalizeToRange.h"
#include <frc2/Timer.h>
using namespace nt;

class RotateToAngleCommand
    : public frc2::CommandHelper<frc2::CommandBase, RotateToAngleCommand> {

 private:
  
  double m_turnInput;
  double m_targetZone;
  double m_targetYaw;
  bool m_tv;
  int m_loopCounter = 0;
  frc2::Timer m_timer;
  DriveSubsystem* m_drive;
  frc2::PIDController m_turningPIDController{
      3.5, 5, .1};
  // using radians_per_second_squared_t =
  //   units::compound_unit<units::radians,
  //                        units::inverse<units::squared<units::second>>>;
  // frc::ProfiledPIDController<units::degrees> m_turningPIDController{
  //     3, 0, 0.2,
  //     {units::degrees_per_second_t(RobotParameters::k_maxYawRate), units::unit_t<radians_per_second_squared_t>(RobotParameters::k_maxYawAccel*MATH_CONSTANTS_PI/180)}};
 public:
  RotateToAngleCommand(DriveSubsystem* driveTrain, double targetYaw, double targetZone){
    m_drive = driveTrain;
    m_targetZone = targetZone;
    m_targetYaw = targetYaw;
    m_timer.Start();
    AddRequirements(m_drive);
    // m_turningPIDController.SetP(frc::SmartDashboard::GetNumber("angle to rotate p", 35));
    // m_turningPIDController.SetI(frc::SmartDashboard::GetNumber("angle to rotate i", 35));
    // m_turningPIDController.SetD(frc::SmartDashboard::GetNumber("angle to rotate d", 35));
    m_turningPIDController.SetIntegratorRange(-5,5);
    frc::SmartDashboard::PutNumber("angle to rotate p", 3.5);
    frc::SmartDashboard::PutNumber("angle to rotate i", 5);
    frc::SmartDashboard::PutNumber("angle to rotate d", .1);
    frc::SmartDashboard::PutNumber("angle to rotate i zone", 5);
  }

  void Initialize() override{
    
    // m_targetYaw = frc::SmartDashboard::GetNumber("angle to rotate ", 35);
    m_turningPIDController.EnableContinuousInput(-180,180);
  }

  void Execute() override{
    m_turnInput = m_drive->GetPose().Rotation().Degrees().to<double>();
    
    double yawRate = m_turningPIDController.Calculate(m_turnInput, m_targetYaw); //pid tune motor controller

    m_drive->Drive(units::meters_per_second_t(0), // set the driveTrain
                   units::meters_per_second_t(0),
                   units::degrees_per_second_t(yawRate),
                   false);
    frc::SmartDashboard::PutNumber("YawRate", yawRate);
    frc::SmartDashboard::PutNumber("m_turnInput", m_turnInput);
    if(fabs(normalizeToRange::RangedDifference(m_turnInput - m_targetYaw,-180, 180)) <= m_targetZone){
      m_loopCounter++;
    }else{
      m_loopCounter = 0;
    }
  }

  void End(bool interrupted) override{
    m_drive->Drive(0_mps,0_mps,0_rpm,false);//STOP DRIVE TRAIN
    printf("rotate to angle time since init %f\n", m_timer.Get().to<double>());
    m_timer.Stop();
  }

  bool IsFinished() override{
    // printf(" - - -- - - - - rotating still-   - -- - -- - --\n");
    return m_loopCounter >=5;
  }
};
