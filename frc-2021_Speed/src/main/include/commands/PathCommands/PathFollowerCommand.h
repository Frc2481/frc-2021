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
#include <units/velocity.h>
#include <units/angle.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <fstream>
#include <frc2/Timer.h>
// #include "Utils/SwerveDrivePathGenerator.h"

class PathFollowerCommand : public frc2::CommandHelper<frc2::CommandBase, PathFollowerCommand> {
 private:
  SwerveDrivePathFollower m_pFollower;
  DriveSubsystem* m_pDriveSubsystem;
  std::ofstream m_File;
  bool m_zero;
  frc2::Timer m_timer;
  std::string m_name;
  double metersToInches = 39.3701;
 public:
  PathFollowerCommand(DriveSubsystem* driveTrain,
                      std::vector<SwerveDrivePathGenerator::waypoint_t> &waypoints, const std::string &name,
                      bool zero = false){
  m_pFollower.generatePath(waypoints, name);
  m_pDriveSubsystem = driveTrain;
  m_zero = zero;
  m_name = name;
  AddRequirements(m_pDriveSubsystem);
  }

  void Initialize() override{
    m_timer.Start();
    // std::remove("home/lvuser/vel.csv");
    // m_File.open("home/lvuser/vel.csv");
    // m_File << " actual x vel, actual y vel, x vel, y vel\n";
    m_pFollower.start();
    if(m_zero){
      m_pDriveSubsystem->ResetOdometry(m_pFollower.getPointPos(0));
    }
    // printf("initialize\n");
    // m_pDrivetrain->ZeroHeading();//starting at first point in the path
    

  } 

  void Execute() override{
    
    m_pFollower.Update(m_pDriveSubsystem->GetPose());
    frc::SmartDashboard::PutNumber("follow path command x", m_pFollower.getPointPos(20).Translation().X().to<double>());
    frc::SmartDashboard::PutNumber("follow path command y", m_pFollower.getPointPos(20).Translation().Y().to<double>());
    frc::SmartDashboard::PutNumber("actual x vel", m_pDriveSubsystem->GetRobotVelocity().vx.to<double>());
    frc::SmartDashboard::PutNumber("actual y vel", m_pDriveSubsystem->GetRobotVelocity().vy.to<double>());
    frc::SmartDashboard::PutNumber("x path vel", m_pFollower.getXVel()*metersToInches);
    frc::SmartDashboard::PutNumber("y path vel", m_pFollower.getYVel()*metersToInches);
    // frc::SmartDashboard::PutNumber("total diff", m_pFollower.getYVel() - m_pDriveSubsystem->GetRobotVelocity().vy.to<double>());
    // m_File << m_pDriveSubsystem->GetRobotVelocity().vx.to<double>() << ",";
    // m_File << m_pDriveSubsystem->GetRobotVelocity().vy.to<double>() << ",";
    // m_File << m_pFollower.getXVel() << ",";
    // m_File << m_pFollower.getYVel() << "\n";
    // frc::SmartDashboard::PutNumber("follow path command yaw", m_pFollower.getPointPos(20).Translation().X().to<double>())
    // printf("path x vel %f, ", m_pFollower.getXVel());
    // printf("path y vel %f, ", m_pFollower.getYVel());
    // printf("actual x vel %f, ", m_pDriveSubsystem->GetRobotVelocity().vx.to<double>());
    // printf("actual y vel %f\n", m_pDriveSubsystem->GetRobotVelocity().vy.to<double>());
    m_pDriveSubsystem->Drive(units::meters_per_second_t(m_pFollower.getXVel()), 
                             units::meters_per_second_t(m_pFollower.getYVel()), 
                             units::degrees_per_second_t(m_pFollower.getYawRate()),
                             true,
                             false);
  }

  void End(bool interrupted) override{
    m_pDriveSubsystem->stop();
    // m_File.close();
    // printf("ended\n");
    printf("path follower time since Initialize %f\n", m_timer.Get().to<double>());
    m_timer.Stop();
  }

  bool IsFinished() override{
    // return false;
    return m_pFollower.isPathFinished();
  }
};

