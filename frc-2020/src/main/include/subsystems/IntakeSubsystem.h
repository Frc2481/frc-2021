#pragma once
#include <frc2/command/SubsystemBase.h>
#include <frc/Solenoid.h>
#include <frc/doubleSolenoid.h>
#include <frc/DigitalInput.h>
#include <ctre/Phoenix.h>

#include "components/VictorMotorController.h"
class IntakeSubsystem : public frc2::SubsystemBase {

    
public:
    IntakeSubsystem();
    /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
   void Periodic() override;
   void setRollerSpeed(double speed);

   void extend();
   void retract(); 
   bool isIntakeExtended();
   double getRollerSpeed();
   
   
private: 
    double m_rollerSpeed;
   
    bool m_intakeExtended;
    
    frc::DoubleSolenoid m_intakeSolenoid;
    VictorMotorController m_rollerMotor;
    
    };
