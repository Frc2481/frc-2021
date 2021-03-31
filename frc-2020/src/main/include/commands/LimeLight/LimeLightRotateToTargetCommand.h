/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "networktables/NetworkTableInstance.h"
#include <sstream>
#include "RobotContainer.h"
#include "subsystems/DriveSubsystem.h"
#include <wpi/math>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc/XboxController.h>
#include <frc/controller/ProfiledPIDController.h>
#include "RobotParameters.h"
#include "components/Joystick2481.h"
#include <frc/Timer.h>
#include "Commands/LimeLight/TurnLimeLightOn.h"
#include "Commands/LimeLight/TurnLimeLightOff.h"
#include <frc2/Timer.h>
using namespace nt;

class 
LimeLightRotateToTargetCommand
    : public frc2::CommandHelper<frc2::CommandBase, LimeLightRotateToTargetCommand> {

 private:
  
  double m_turnInput;
  double m_targetZone;
  int m_targetLostCounter = 0;
  int m_targetZoneConter = 0;
  double m_targetAngle = 0;
  bool m_tv;
  frc::Timer m_timer;
  DriveSubsystem* m_drive;
  frc2::PIDController m_turningPIDController{
      RobotParameters::k_limeLightP, RobotParameters::k_limeLightI, RobotParameters::k_limeLightD};//3.5, 0, 0};
  
  // frc::ProfiledPIDController<units::radians> m_turningPIDController{
  //         3, 0, 0.2,
  //         AutoConstants::kThetaControllerConstraints};
 public:
  LimeLightRotateToTargetCommand(DriveSubsystem* driveTrain, double targetZone){
    m_drive = driveTrain;
    m_targetZone = targetZone;
    AddRequirements(m_drive);
  }

  void Initialize() override{
    TurnLimeLightOn();
    m_timer.Start();
    // m_turningPIDController.
    m_turningPIDController.EnableContinuousInput(-180,180);
    m_turningPIDController.SetP(frc::SmartDashboard::GetNumber("limeLight P", RobotParameters::k_limeLightP));
    m_turningPIDController.SetI(frc::SmartDashboard::GetNumber("limeLight I", RobotParameters::k_limeLightI));
    m_turningPIDController.SetD(frc::SmartDashboard::GetNumber("limeLight D", RobotParameters::k_limeLightD));
    m_turningPIDController.SetIntegratorRange(-frc::SmartDashboard::GetNumber("limeLight Izone", RobotParameters::k_limeLightIZone),frc::SmartDashboard::GetNumber("limeLight Izone", RobotParameters::k_limeLightIZone));
    m_turnInput = NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0);
    double currentAngle = m_drive->GetPose().Rotation().Degrees().to<double>();
    m_targetAngle = currentAngle - m_turnInput;
  }

  void Execute() override{
     //get target angle
    double m_turnInput = m_drive->GetPose().Rotation().Degrees().to<double>();
    
    // m_targetAngle = currentAngle - m_turnInput;
    double yawRate = m_turningPIDController.Calculate(m_turnInput, m_targetAngle);//m_targetAngle //pid tune motor controller
    m_tv = (bool)NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv",0); // bool target visable

    m_drive->Drive(units::meters_per_second_t(0), // set the driveTrain
                   units::meters_per_second_t(0),
                   units::degrees_per_second_t(yawRate),//*2.2
                   false);
    
    frc::SmartDashboard::PutNumber("YawRate", yawRate);
    frc::SmartDashboard::PutNumber("m_turnInput", m_turnInput);
    if(fabs(m_turnInput)< m_targetZone){//counter for target zone
      m_targetZoneConter++;
      m_turnInput = NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0);
      double currentAngle = m_drive->GetPose().Rotation().Degrees().to<double>();
      // if(m_turnInput < currentAngle){
        m_targetAngle = currentAngle - m_turnInput ;
      // }else{
      //   m_targetAngle = m_turnInput - currentAngle;
      // }
    }else{
      m_targetZoneConter = 0;
    }
    if(!m_tv){// counter for lost target
      m_targetLostCounter++;
    }else{
      m_targetLostCounter = 0;
    }
  }

  void End(bool interrupted) override{
    m_drive->Drive(0_mps,0_mps,0_rpm,false);//STOP DRIVE TRAIN
    printf("lime light rotate command time since init%f\n",m_timer.Get());
    m_timer.Stop();
    TurnLimeLightOff();
  }

  bool IsFinished() override{
    return m_targetZoneConter >= 3||m_targetLostCounter >= 3;//||m_timer.Get() > 2;
  }
};

