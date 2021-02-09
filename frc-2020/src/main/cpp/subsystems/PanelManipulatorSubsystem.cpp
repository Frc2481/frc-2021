/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/PanelManipulatorSubsystem.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "Constants.h"
#include <frc/DriverStation.h>

PanelManipulatorSubsystem::PanelManipulatorSubsystem() :
        m_colorSensor(frc::I2C::Port::kOnboard),
        m_colorMatcher(),
        m_manipulatorSolenoid(SolenoidPorts::kManipulatorSolenoidPort, SolenoidPorts::kManipulatorSolenoidReversePort),
        m_manipulatorMotor(TalonIDs::kManipulatorMotorID, "manipulatorMotor")
        {
    m_colorMatcher.AddColorMatch(kBlueTarget);
    m_colorMatcher.AddColorMatch(kGreenTarget);
    m_colorMatcher.AddColorMatch(kRedTarget);
    m_colorMatcher.AddColorMatch(kYellowTarget);
    m_prevColor = UNKNOWN;
    m_wheelPos = 0.0;
    m_colorToRotateTo = UNKNOWN;
}

// This method will be called once per scheduler run
void PanelManipulatorSubsystem::Periodic(){
  std::string gameData;
    gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
    if(gameData.length() > 0)
    {
      switch (gameData[0])
      {
        case 'B' :
          m_colorToRotateTo = BLUE;
          break;
        case 'G' :
          m_colorToRotateTo = GREEN;
          break;
        case 'R' :
          m_colorToRotateTo = RED;
          break;
        case 'Y' :
          m_colorToRotateTo = YELLOW;
          break;
        default :
          //This is corrupt data
          break;
      }
    } else {
      //Code for no data received yet
    }
  Color_t color = getMatchedColor();
  // frc::Color rawColor = getRawColor();
  // printf("Current Color %d\n", color);
  // printf("Raw Color R %f G %f B %f\n", rawColor.red, rawColor.green, rawColor.blue);
  if(m_prevColor != UNKNOWN){
    if(color < m_prevColor && color != UNKNOWN){
      // we have crossed 0
      m_wheelPos += (4+((int)color-m_prevColor))/8.0;
    } else if(color - m_prevColor >= 1 && color != UNKNOWN){
        // if we didn't cross 0 
        m_wheelPos += (color - m_prevColor)/8.0;
    }
  }
  if(color != UNKNOWN){
    m_prevColor = color;
  }
  // printf("panel rotations %f\n", m_wheelPos);
  frc::SmartDashboard::PutNumber("Panel Rotations", m_wheelPos);
} 

frc::Color PanelManipulatorSubsystem::getRawColor(){
    frc::Color color = m_colorSensor.GetColor();
    return color;
}

void PanelManipulatorSubsystem::zeroPanelManipulator(){
  m_wheelPos = 0;
}
float PanelManipulatorSubsystem::getPanelPos(){
  return m_wheelPos;
}
bool PanelManipulatorSubsystem::isPanelManipulatorOn(){
  return m_turning;
}
void PanelManipulatorSubsystem::setExtended(bool extended){
  m_manipulatorSolenoid.Set(frc::DoubleSolenoid::kForward);
}
void PanelManipulatorSubsystem::setMotorSpeed(double speed){
  m_manipulatorSolenoid.Set(frc::DoubleSolenoid::kReverse);
}
int PanelManipulatorSubsystem::getDistanceToColor(Color_t color){
  int raw =  (int)color - (int)m_prevColor;
  if(color == UNKNOWN || m_prevColor == UNKNOWN){
    return 4;
  }
  if(raw == 3){
    raw = -1;
  }else if(raw == -3){
    raw = 1;
  }
  return raw;
}
PanelManipulatorSubsystem::Color_t PanelManipulatorSubsystem::getTargetColor(){
  return m_colorToRotateTo;
}
PanelManipulatorSubsystem::Color_t PanelManipulatorSubsystem::getMatchedColor(){
    double confidence = 0.0;
    frc::Color detectedColor = getRawColor();
    frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);
    // printf("confidence %f \n", confidence);
    Color_t color = UNKNOWN;
    if (matchedColor == kBlueTarget && confidence > 0.968) {
      color = BLUE;
    } else if (matchedColor == kRedTarget && confidence > 0.968) {
      color = RED;
    } else if (matchedColor == kGreenTarget && confidence > 0.968) {
      color = GREEN;
    } else if (matchedColor == kYellowTarget && confidence > 0.968) {
      color = YELLOW;

    }
    return color;
}
