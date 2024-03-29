/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "components/VictorMotorController.h"
#include <frc/DigitalInput.h>
#include "Constants.h"
class IndexerSubsystem : public frc2::SubsystemBase {
 public:
  IndexerSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();
  void setIndexerSpeed(double speed);
  double getIndexerSpeed();
  bool isBallInTrench();
  bool isBallInBottomBeamBreak();
 private:
  double m_indexerSpeed;
  
  VictorMotorController m_indexerMotor;
  frc::DigitalInput m_trenchBeamBreak;
  frc::DigitalInput m_bottomBeamBreak;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
