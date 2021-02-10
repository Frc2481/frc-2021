#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"
#include "RobotParameters.h"

    IntakeSubsystem::IntakeSubsystem() :
    m_rollerSpeed(0),
    m_intakeExtended(false),
    m_intakeSolenoid(0, SolenoidPorts::kIntakeSolenoidPort,SolenoidPorts::kIntakeSolenoidReversePort),
    m_rollerMotor(VictorIDs::kIntakeRollerMotorID,"RollerMotor")
    {
        
    }

    void IntakeSubsystem::Periodic(){
    }
    void IntakeSubsystem::setRollerSpeed(double speed){
        m_rollerSpeed = speed;
        m_rollerMotor.Set(speed);
    }
    
    void IntakeSubsystem::extend(){
        // printf("----------extend----------\n");
        m_intakeExtended = true;
        m_intakeSolenoid.Set(m_intakeSolenoid.kForward);//frc::DoubleSolenoid::Value::kReverse);//TODO uncomment
    }
    void IntakeSubsystem::retract(){
        m_intakeExtended = false;
        // printf("----------retract----------\n");
        m_intakeSolenoid.Set(m_intakeSolenoid.kReverse);//Set(frc::DoubleSolenoid::Value::kForward);//TODO uncomment
    }
    bool IntakeSubsystem::isIntakeExtended(){
        return m_intakeExtended;
    }
    double IntakeSubsystem::getRollerSpeed(){
        return m_rollerSpeed;
    }
    