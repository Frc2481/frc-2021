/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include "subsystems/PanelManipulatorSubsystem.h"

class ReadColorSensorCommand
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 ReadColorSensorCommand> {
 private: 
    PanelManipulatorSubsystem* m_colorSensor;

 public:
  ReadColorSensorCommand(PanelManipulatorSubsystem* colorSensor){
    m_colorSensor = colorSensor;
    AddRequirements(m_colorSensor);
  }
  void Initialize() override {
    frc::Color rawColor = m_colorSensor->getRawColor();
    frc::SmartDashboard::PutNumber("Red",rawColor.red);
    frc::SmartDashboard::PutNumber("Green", rawColor.green);
    frc::SmartDashboard::PutNumber("Blue", rawColor.blue);
    PanelManipulatorSubsystem::Color_t matchedColor = m_colorSensor->getMatchedColor();

    if (matchedColor == PanelManipulatorSubsystem::BLUE) {
      frc::SmartDashboard::PutString("Matched Color", "BLUE");
    } else if (matchedColor == PanelManipulatorSubsystem::RED) {
      frc::SmartDashboard::PutString("Matched Color", "RED");
    } else if (matchedColor == PanelManipulatorSubsystem::GREEN) {
      frc::SmartDashboard::PutString("Matched Color", "GREEN");
    } else if (matchedColor == PanelManipulatorSubsystem::YELLOW) {
      frc::SmartDashboard::PutString("Matched Color", "YELLOW");
    } else if (matchedColor == PanelManipulatorSubsystem::UNKNOWN) {
    frc::SmartDashboard::PutString("Matched Color", "UNKNOWN");
    }

  }
};
