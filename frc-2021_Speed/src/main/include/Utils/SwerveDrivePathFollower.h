/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <vector>
#include <limits>
#include <algorithm>
#include "RobotParameters.h"
#include "Utils/SwerveDrivePathGenerator.h"
#include "Utils/Sign.h"
#include "Utils/PIDVAController.h"
#include "subsystems/DriveSubsystem.h"
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include "Utils/PoseDot2D.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include "Utils/MathConstants.h"
#include <frc/controller/PIDController.h>
#include <frc/geometry/Pose2d.h>
#include "Utils/CoordinateTranslation.h"
#include <frc/controller/ProfiledPIDController.h>

class SwerveDrivePathFollower {
 public:
  SwerveDrivePathFollower();


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void generatePath(std::vector<SwerveDrivePathGenerator::waypoint_t> &waypoints, const std::string &name);
  void Update(frc::Pose2d pose);
  double getXVel();
  double getYVel();
  double getYawRate();
  void start();
  int closestPointIndex();
  int lookAheadIndex();
  bool isPathFinished();
  frc::Pose2d getPointPos(int index);
  // bool targetPosFound();
  // void setTargetPos();
  void stop(bool interrupted);

 private:
  int m_metersToInches = 39.3701;
	std::vector<SwerveDrivePathGenerator::finalPathPoint_t> m_path;
  std::vector<double> m_xVect;
  std::vector<double> m_yVect;
  

  bool m_pathFinished = false;
    bool m_lastPointReached;
    double m_distToEnd;
    double m_targetZone;

    int m_lookAheadIndex = 0;
  int m_pathIndex = 0;


	double m_lookAhead = .6;
	int m_maxDrifting = 5;

  double m_xVel = 0;
  double m_yVel = 0;
  double m_yawRate = 0;

  double m_time;

	std::ofstream m_File;
	frc2::PIDController m_turningPIDController{
      8, 0, 0};
};
