    //TODO: Pulled from 2020, still needs dual-intake compatability but CAD hasn't figured out how that needs to be done yet. -Sean H.
    //TODO Copied this from 2020 because intake supposedly similar, INCOMPLETE. -Sean H.
    
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <ctre/Phoenix.h>

#include "components/VictorMotorController.h"
#include <frc/DigitalInput.h>

class IntakeSubsystem : public frc2::SubsystemBase {
    
public:
    IntakeSubsystem();
    /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
   void Periodic() override;
   void setLeftSpeed(double speed);
   void setRightSpeed(double speed);
   double getLeftSpeed();
   double getRightSpeed();
   double getBeamBreak();
   
private: 
    double leftSpeed;
    double rightSpeed;
    frc::DigitalInput* m_beamBreak;
    VictorMotorController* m_rightMotor;
    VictorMotorController* m_leftMotor;
    
    };
