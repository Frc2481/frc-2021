/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/doubleSolenoid.h>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include "components/TalonSRXMotorController.h"


class PanelManipulatorSubsystem : public frc2::SubsystemBase {
 public:
  typedef enum Color{
    BLUE = 2, GREEN = 1, RED = 0, YELLOW = 3, UNKNOWN = 4
  } Color_t;

  PanelManipulatorSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

  frc::Color getRawColor();

  Color_t getMatchedColor();

  Color_t getTargetColor();
  void zeroPanelManipulator();
  float getPanelPos();
  bool isPanelManipulatorOn();
  void setExtended(bool extended);
  void setMotorSpeed(double speed);
  int getDistanceToColor(Color_t color);
  

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::ColorSensorV3 m_colorSensor;
  rev::ColorMatch m_colorMatcher;

  Color_t m_prevColor;
  Color_t m_colorToRotateTo;
  frc::DoubleSolenoid m_manipulatorSolenoid;
  TalonSRXMotorController m_manipulatorMotor;

  float m_wheelPos;
  bool m_turning;
  

  static constexpr frc::Color kBlueTarget = frc::Color(0.1251, 0.4069, 0.4681);
  static constexpr frc::Color kGreenTarget = frc::Color(0.1798, 0.5594, 0.2606);
  static constexpr frc::Color kRedTarget = frc::Color(0.5018, 0.3495, 0.1486);
  static constexpr frc::Color kYellowTarget = frc::Color(0.3219, 0.5514, 0.1266);

};
