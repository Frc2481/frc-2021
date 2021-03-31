/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "SwerveDrivePathGenerator.h"
#include "Utils/NormalizeToRange.h"
#include <iostream>
#include <fstream>
#pragma once

class CoordinateTranslation {
 public:
  CoordinateTranslation(){

  }
  
  static void NolanToWPILib(std::vector<SwerveDrivePathGenerator::finalPathPoint_t> nolanPath,std::vector<SwerveDrivePathGenerator::finalPathPoint_t> &wpiPath){
    std::ofstream m_File;
    std::remove("home/lvuser/NolanToWPILib.csv");
		m_File.open("home/lvuser/NolanToWPILib.csv");
		m_File <<"xPos (m), yPos (m), yaw (deg), vel (m/s),   heading, accel(m/s/s),  dist(m)\n";
    wpiPath.clear();
    double inchesToMeters = .0254;
    for(int i = 0; i < (int)nolanPath.size();i++){
      // printf("NolanToWPILib cycle: %d\n", i);
      SwerveDrivePathGenerator::finalPathPoint_t wpiPoint = nolanPath[i];
      wpiPoint.xPos = nolanPath[i].yPos*inchesToMeters;
      wpiPoint.yPos = -nolanPath[i].xPos*inchesToMeters;
      wpiPoint.yaw = normalizeToRange::NormalizeToRange(nolanPath[i].yaw-90,-180,180, true);
      wpiPoint.heading = normalizeToRange::NormalizeToRange(nolanPath[i].heading-90,-180,180, true);
      wpiPoint.vel = nolanPath[i].vel*inchesToMeters;
      wpiPoint.accel = nolanPath[i].accel*inchesToMeters;
      wpiPoint.dist = nolanPath[i].dist*inchesToMeters;
      wpiPath.push_back(wpiPoint);
      m_File << wpiPoint.xPos << ", ";
      m_File << wpiPoint.yPos << ", ";
      m_File << wpiPoint.yaw << ", ";
      m_File << wpiPoint.vel << ", ";
      m_File << wpiPoint.heading << ", ";
      m_File << wpiPoint.accel << ", ";
      m_File << wpiPoint.dist << ", ";
      m_File << "\n";
    }
  }
  static void WPILibToNolan(std::vector<SwerveDrivePathGenerator::waypoint_t> wpiPath, std::vector<SwerveDrivePathGenerator::waypoint_t> &nolanPath){
    double metersToInches = 39.3701;
    // double metersToInches = 12.0;// inches to feet
    std::ofstream m_File;
    std::remove("home/lvuser/WPILibToNolan.csv");
		m_File.open("home/lvuser/WPILibToNolan.csv");
		m_File <<"xPos (m), yPos (m), yaw (deg), speed (m/s),   radCurve\n";
    nolanPath.clear();
    for(int i = 0; i < (int)wpiPath.size();i++){
      // printf("WPILibToNolan cycle: %d\n", i);
      SwerveDrivePathGenerator::waypoint_t nolanPoint = wpiPath[i];
      nolanPoint.xPos = -wpiPath[i].yPos*metersToInches;
      nolanPoint.yPos = wpiPath[i].xPos*metersToInches;
      nolanPoint.speed = wpiPath[i].speed*metersToInches;
      nolanPoint.radCurve = wpiPath[i].radCurve*metersToInches;
      nolanPoint.yaw = normalizeToRange::NormalizeToRange(wpiPath[i].yaw+90,-180,180, true);
      nolanPath.push_back(nolanPoint);
      m_File << nolanPoint.xPos << ", ";
      m_File << nolanPoint.yPos << ", ";
      m_File << nolanPoint.yaw << ", ";
      m_File << nolanPoint.speed << ", ";
      m_File << nolanPoint.radCurve << ", ";
      m_File << "\n";
    }
  } 
  private:
    
};
