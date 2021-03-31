
    
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <ctre/Phoenix.h>

#include "components/TalonSRXMotorController.h"
// #include <frc/DigitalInput.h>
#include <frc/Servo.h>
class IntakeSubsystem : public frc2::SubsystemBase {
    
public:
    IntakeSubsystem();
    /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
   void Periodic() override;
   void setBIntakeSpeed(double speed);
   void setAIntakeSpeed(double speed);
   void setAIntakeCurrent(double current);
   void setBIntakeCurrent(double current);
   double getIntakeACurrent();
   double getIntakeBCurrent();

   double getBIntakeSpeed();
   double getAIntakeSpeed();
   
   double getServoAngle();
   void setServoAngle(double angle);
private: 
    TalonSRXMotorController* m_aIntakeMotor;
    TalonSRXMotorController* m_bIntakeMotor;
    frc::Servo* m_servo;
    };
