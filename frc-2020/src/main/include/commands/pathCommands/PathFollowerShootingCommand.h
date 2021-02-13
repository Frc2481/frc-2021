/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "Utils/SwerveDrivePathFollower.h"
#include <vector>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/FeederSubsystem.h"
#include "networktables/NetworkTableInstance.h"

#include <units/velocity.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <fstream>
#include <frc2/Timer.h>

class PathFollowerShootingCommand : public frc2::CommandHelper<frc2::CommandBase, PathFollowerShootingCommand> {
 private:
  SwerveDrivePathFollower m_pFollower;
  DriveSubsystem* m_pDriveSubsystem;
  ShooterSubsystem* m_pShooterSubsystem;
  FeederSubsystem* m_pFeederSubsystem;
  std::ofstream m_File;
  bool m_zero;
  double m_zone;
  frc2::Timer m_timer;
  std::string m_name;
  double inputConstant = 2.2;
  frc::ProfiledPIDController<units::radians> m_turningPIDController{
          3, 0, 0.2,//5//RobotParameters::k_limeLightDriveP, RobotParameters::k_limeLightDriveI, RobotParameters::k_limeLightDriveD,
          AutoConstants::kThetaControllerConstraints};
 public:
  PathFollowerShootingCommand(DriveSubsystem* driveTrain,
                      ShooterSubsystem* shooter,
                      FeederSubsystem* feeder,
                      double zone,
                      std::vector<SwerveDrivePathGenerator::waypoint_t> &waypoints, const std::string &name,
                      bool zero = false){
  m_pFollower.generatePath(waypoints, name);
  m_pDriveSubsystem = driveTrain;
  m_pShooterSubsystem = shooter;
  m_pFeederSubsystem = feeder;
  m_zone = zone;
  m_zero = zero;
  m_name = name;
  AddRequirements(m_pDriveSubsystem);
  AddRequirements(m_pShooterSubsystem);
  AddRequirements(m_pFeederSubsystem);
  }

  void Initialize() override{
    m_timer.Start();
    m_pFollower.start();
    if(m_zero){
      m_pDriveSubsystem->ResetOdometry(m_pFollower.getPointPos(0));
    }
    m_pShooterSubsystem->startShooter();
    m_pShooterSubsystem->setCloseShot(false);
    m_pShooterSubsystem->setShooterSpeed(ShooterConstants::kDefaultShooterShortSpeed);
  } 

  void Execute() override{
    
    m_pFollower.Update(m_pDriveSubsystem->GetPose());

    double yawRate = m_pFollower.getYawRate()*wpi::math::pi/180;
    double m_turnInput = NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0)*wpi::math::pi/180; //get target angle
    bool tv = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv",0);
    if(tv){
      yawRate = m_turningPIDController.Calculate(units::radian_t(m_turnInput))*inputConstant; //
      //m_pShooterSubsystem->autoShootSpeed();
    }
    m_pDriveSubsystem->Drive(units::meters_per_second_t(m_pFollower.getXVel()), 
                             units::meters_per_second_t(m_pFollower.getYVel()), 
                             units::radians_per_second_t(yawRate),
                             true,
                             false);
    if(m_turnInput < m_zone && tv){//m_pShooterSubsystem->isShooterOnTarget() && 
      m_pFeederSubsystem->setFeederSpeed(FeederConstants::kShootingFeederSpeed);
    }
  }

  void End(bool interrupted) override{
    m_pDriveSubsystem->stop();
    m_pShooterSubsystem->stopShooter();
    m_pFeederSubsystem->setFeederSpeed(0.0);
    printf("path follower time since Initialize %f\n", m_timer.Get().to<double>());
    m_timer.Stop();
  }

  bool IsFinished() override{
    return m_pFollower.isPathFinished();
  }
};

