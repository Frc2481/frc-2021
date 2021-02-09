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
using namespace nt;

class LimeLightSwerveDriveRotateToTargetCommand
    : public frc2::CommandHelper<frc2::CommandBase, LimeLightSwerveDriveRotateToTargetCommand> {

 private:
  
  double m_turnInput;
  double m_targetZone = 1;
  int m_targetLostCounter = 0;
  int m_targetZoneConter = 0;
  bool m_autoSkew;
  bool m_tunePID;
  bool m_tv;
  DriveSubsystem* m_drive;
  Joystick2481* m_driverController;
  frc2::PIDController m_drivePIDController{
      1, 0, 0};
  frc::ProfiledPIDController<units::radians> m_turningPIDController{
          RobotParameters::k_limeLightDriveP, RobotParameters::k_limeLightDriveI, RobotParameters::k_limeLightDriveD,
          AutoConstants::kThetaControllerConstraints};
 public:
  LimeLightSwerveDriveRotateToTargetCommand(DriveSubsystem* driveTrain, Joystick2481* driveController, bool autoSkew, bool tunePID = false){
    m_drive = driveTrain;
    m_driverController = driveController;
    
    AddRequirements(m_drive);
  }

  void Initialize() override{
    // m_turningPIDController.SetP(frc::SmartDashboard::GetNumber("limeLight P", RobotParameters::k_limeLightDriveP));
    // m_turningPIDController.SetI(frc::SmartDashboard::GetNumber("limeLight I", RobotParameters::k_limeLightDriveI));
    // m_turningPIDController.SetD(frc::SmartDashboard::GetNumber("limeLight D", RobotParameters::k_limeLightDriveD));
    m_targetZone = frc::SmartDashboard::GetNumber("Target Zone", .5)*wpi::math::pi/180; // update target zone ofset +-
  }

  void Execute() override{
    double xleftHand = 0.0; //robot x and y
    double yleftHand = 0.0;
      xleftHand = -m_driverController->GetRawAxis(0);//left xbox joystick
        if(fabs(xleftHand) <=0.15){//dead zone
          xleftHand = 0.0;
        }
      yleftHand = m_driverController->GetRawAxis(1);//right xbox joystick
        if(fabs(yleftHand) <=0.15){//dead zone
          yleftHand = 0.0;
      }
    m_turnInput = NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0)*wpi::math::pi/180; //get target angle
    auto yawRate = m_turningPIDController.Calculate(units::radian_t((m_turnInput))); //pid tune motor controller

    m_tv = (bool)NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv",0); // bool target visable
    
    
    
    if(fabs(m_turnInput)< m_targetZone){// set turning to zero if in target zone
      m_turnInput = 0;
    }
    // printf("Limelight Yaw: %.01f",yawRate);
    m_drive->Drive(units::meters_per_second_t(-xleftHand), // set the driveTrain
                   units::meters_per_second_t(-yleftHand),
                   units::radians_per_second_t(yawRate),
                   false);
    frc::SmartDashboard::PutNumber("YawRate", yawRate);

    if(fabs(m_turnInput)< m_targetZone){//counter for target zone
      m_targetZoneConter++;
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
    frc::SmartDashboard::PutNumber("Target Zone", m_targetZone*180/wpi::math::pi);
    m_drive->Drive(0_mps,0_mps,units::radians_per_second_t(0),false);//STOP DRIVE TRAIN
  }

  bool IsFinished() override{
    return false;
  }
};
