/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "components/TalonFXMotorController.h"
#include "Constants.h"
#include "RobotParameters.h"
class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  ClimberSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();
  void climb(double percent);
  void stop();
  void grabBar();
  void zeroPos();
  double getPos();
  void goToPos(double pos);
  void pid(double p, double i, double d, double f, double izone);
  bool climberReady();
 private:
 double m_targetHeight = 0;
 double m_rotateToHeight;
 double m_climbPos = 0;
 bool m_climberReady = true;
 TalonFXMotorController m_climber;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
