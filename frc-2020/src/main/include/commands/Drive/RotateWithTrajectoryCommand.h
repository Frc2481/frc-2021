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
#include "Utils/TrajectoryGenerator1D.h"
#include "Utils/NormalizeToRange.h"
#include <frc2/Timer.h>
#include "networktables/NetworkTableInstance.h"
using namespace nt;

class RotateWithTrajectoryCommand
    : public frc2::CommandHelper<frc2::CommandBase, RotateWithTrajectoryCommand> {

 private:
  TrajectoryGenerator1D* trajectoryGenerator;
  
  double m_turnInput;
  double m_targetZone;
  double m_targetYaw;
  bool m_tv;
  double m_pathIndex;
  double m_closestDistance;
  bool m_limeLight;
  frc2::Timer m_timer;
  frc2::PIDController m_turningPIDController{
      3.5, 5, .1};
  DriveSubsystem* m_drive;
  std::vector<TrajectoryGenerator1D::waypoint_t> path;
  std::vector<TrajectoryGenerator1D::finalPathPoint_t> m_path;
  std::ofstream m_File;

 public:
  RotateWithTrajectoryCommand(DriveSubsystem* driveTrain, double targetYaw, double targetZone, bool limeLight = false){
    m_drive = driveTrain;
    m_targetZone = targetZone;
    m_targetYaw = targetYaw;
    m_limeLight = limeLight;
    AddRequirements(m_drive);
    frc::SmartDashboard::PutNumber("angle to rotate",0);
  }

  void Initialize() override{
    m_File.open("home/lvuser/ActualYawPath.csv",std::ofstream::out);
    m_timer.Start();
    
    m_pathIndex = 0;
    m_turnInput = m_drive->GetPose().Rotation().Degrees().to<double>();
    double currentYawRate = m_drive->GetTurnRate();
    if(m_limeLight){
      m_turnInput = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0);
      double currentAngle = m_drive->GetPose().Rotation().Degrees().to<double>();
      if(m_turnInput < currentAngle){
        m_targetYaw = currentAngle - m_turnInput;
      }else{
        m_targetYaw = m_turnInput - currentAngle;
      }
    }
    double yawDiff = normalizeToRange::NormalizeToRange(normalizeToRange::RangedDifference(m_targetYaw - m_turnInput,-180,180)/2+m_turnInput,-180,180,true);
    
    path.clear();
    path.push_back(TrajectoryGenerator1D::waypoint_t{m_turnInput,currentYawRate});
    path.push_back(TrajectoryGenerator1D::waypoint_t{yawDiff,RobotParameters::k_maxYawRate});
    path.push_back(TrajectoryGenerator1D::waypoint_t{m_targetYaw,0});
    trajectoryGenerator = new TrajectoryGenerator1D(path,50,RobotParameters::k_maxYawRate,RobotParameters::k_maxYawAccel, RobotParameters::k_maxYawDeccel);
    
    trajectoryGenerator->setIsContinous(true, -180,180);
    trajectoryGenerator->generatePath();
    trajectoryGenerator->writePathToCSV();
    // trajectoryGenerator->setPathFilename("rotating with trajectory path.csv");
    m_turningPIDController.EnableContinuousInput(-180,180);
    m_path.clear();
    m_path = trajectoryGenerator->getFinalPath();
    m_File << "path yaw, path vel, robot yaw, diff in yaw\n";
  }

  void Execute() override{
    m_turnInput = m_drive->GetPose().Rotation().Degrees().to<double>();
    m_closestDistance = std::numeric_limits<double>::infinity();
    for(int j = m_pathIndex; j < (int)m_path.size(); j++){
			
			
			double diff = fabs(normalizeToRange::RangedDifference(m_path[j].pos-m_turnInput, -180,180));
      printf("point looking at: %d %f %f %f %f\n", j, m_path[j].pos, m_turnInput, m_path[j].vel, diff);
			if(diff < m_closestDistance){
				if(fabs(m_path[j].vel) > RobotParameters::k_minYawRate){
					m_pathIndex = j;
					m_closestDistance = diff;
        }
			}else{
        break;
      }
		}
    m_File << m_path[m_pathIndex].pos <<","
           << m_path[m_pathIndex].vel << ","
           << m_turnInput <<","
           << m_closestDistance <<"\n";
    double output = m_path[m_pathIndex].vel;
    printf("vel %f\n", output);
    m_drive->Drive(units::meters_per_second_t(0), // set the driveTrain
                   units::meters_per_second_t(0),
                   units::degrees_per_second_t(output),
                   true,false);
    frc::SmartDashboard::PutNumber("target traject Yaw", m_path[m_pathIndex].pos);
    frc::SmartDashboard::PutNumber("Actual rotate traject yaw", m_turnInput);
    frc::SmartDashboard::PutNumber("traject YawRate", output);
    // frc::SmartDashboard::PutNumber("m_turnInput", m_turnInput);
  }

  void End(bool interrupted) override{
    printf("rotate to angle with torjecttory time since init %f\n", m_timer.Get().to<double>());
    m_drive->Drive(0_mps,0_mps,0_rpm,false);//STOP DRIVE TRAIN
    m_timer.Stop();
    m_File.close();
  }

  bool IsFinished() override{
    return m_pathIndex >= (int)m_path.size()-1 && m_closestDistance < m_targetZone ;
  }
};

