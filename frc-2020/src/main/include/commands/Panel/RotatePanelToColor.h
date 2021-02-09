/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/PanelManipulatorSubsystem.h"
#include "Constants.h"


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RotatePanelToColor
    : public frc2::CommandHelper<frc2::CommandBase, RotatePanelToColor> {
 
 private:
  PanelManipulatorSubsystem* m_panelManipulator;
  PanelManipulatorSubsystem::Color_t m_targetColor;
  bool m_finished;

 public:
  RotatePanelToColor(PanelManipulatorSubsystem* panelManipulator) {
    m_panelManipulator = panelManipulator;
    m_finished = false;
    AddRequirements(m_panelManipulator);
  }

  void Initialize() override{
    m_targetColor = m_panelManipulator->getTargetColor();
    m_panelManipulator->zeroPanelManipulator();
    m_panelManipulator->setExtended(true);
    m_panelManipulator->setMotorSpeed(PannelConstants::kDefaultPannelSpeed);
    }
  void Execute() override{
    int distance = m_panelManipulator->getDistanceToColor(m_targetColor);
    if(distance != 4  && distance > 0){
      m_panelManipulator->setMotorSpeed(PannelConstants::kDefaultPannelSpeed);
    }else if(distance != 4){
      m_panelManipulator->setMotorSpeed(-PannelConstants::kDefaultPannelSpeed);
    }
  }

  void End(bool interrupted) override{
    m_panelManipulator->setMotorSpeed(0.0);
  }

  bool IsFinished() override{
    return m_panelManipulator->getMatchedColor() == m_targetColor;
  }
};
