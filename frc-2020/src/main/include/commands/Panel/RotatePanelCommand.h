
//TODO: incomplete! Subsystem incapable of rotating control panel.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/PanelManipulatorSubsystem.h"
#include "frc/smartdashboard/SmartDashboard.h"

class RotatePanelCommand
    : public frc2::CommandHelper<frc2::CommandBase, RotatePanelCommand> {
  
 private:
  PanelManipulatorSubsystem* m_panelManipulator;
  bool m_finished;
 
 public:
  RotatePanelCommand(PanelManipulatorSubsystem* panelManipulator) {
    m_panelManipulator = panelManipulator;
    m_finished = false;
    AddRequirements(m_panelManipulator);
  }

  bool RunsWhenDisabled() const override{
    return true;
  }

  void Initialize() override{
    m_panelManipulator->zeroPanelManipulator();
    m_panelManipulator->setExtended(true);
    m_panelManipulator->setMotorSpeed(1.0);
    if(!m_panelManipulator->isPanelManipulatorOn()){
      m_finished = true;
    }
    frc::SmartDashboard::PutBoolean("Pannel Rotated 3 times", false);
  }

  void Execute() override{
    frc::SmartDashboard::PutNumber("Panel Position", m_panelManipulator->getPanelPos());
    
    // printf("panel rots: %f\n", m_panelManipulator->getPanelPos());
   if (m_panelManipulator->getPanelPos() >= 3.0){
      // printf("here\n");
      m_finished = true;
    }
  }

  void End(bool interrupted) override{
    frc::SmartDashboard::PutBoolean("Pannel Rotated 3 times", true);
    m_panelManipulator->setExtended(false);
    m_panelManipulator->setMotorSpeed(0.0);
  }

  bool IsFinished() override{
    return m_finished;
  }
};
