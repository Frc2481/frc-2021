/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Solenoid.h>
#include <ctre/Phoenix.h>
#include "components/TalonFXMotorController.h"
#include <frc/DigitalInput.h>

class FeederSubsystem : public frc2::SubsystemBase {
 public:
  FeederSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();
  
  int getBallCount();
  void setFeederSpeed(double speed); 
  double getFeederPos();
  void setFeederPos(double pos);
  bool isBallInTopBeamBreak();
  bool isBallInMidBeamBreak();
  bool getPrevBottomState();
  void zeroPos();
  void zeroBallCount();
  void setBallCount(int count);
  bool isFeederReady();
  bool isMidPrevBeam();
  double getFeederSetSpeed();
  void setSensorPos(double pos);
  void tunePID(double p,double i,double d,double f,double izone);
 private:
  bool m_midBeamPrevState;
  bool m_midBeamState;
  bool m_topBeamPrevState;
  bool m_topBeamState;
  bool m_manualControl;
  double m_feederSpeed;
  TalonFXMotorController m_feeder;
  
  frc::DigitalInput m_midBeamBreak;
  frc::DigitalInput m_topBeamBreak;
  int m_ballCount;
  double m_range = .1;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
