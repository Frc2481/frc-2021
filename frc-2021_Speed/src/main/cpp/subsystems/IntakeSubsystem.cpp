    //TODO Copied this from 2020 because intake supposedly similar, INCOMPLETE. -Sean H.

#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"
#include "RobotParameters.h"
#include "Constants.h"
    IntakeSubsystem::IntakeSubsystem() :
    bIntakeSpeed(0.0),
    aIntakeSpeed(0.0)
    {
        m_servo = new frc::Servo(0);
        m_aIntakeMotor = new VictorMotorController(VictorIDs::kAIntakeID, "Front Intake Motor");
        m_aIntakeMotor->ConfigFactoryDefault();
        m_bIntakeMotor = new VictorMotorController(VictorIDs::kBIntakeID,"Left Intake Motor");
        m_bIntakeMotor->ConfigFactoryDefault();
    }

    void IntakeSubsystem::Periodic(){
    }
    void IntakeSubsystem::setBIntakeSpeed(double speed){
        bIntakeSpeed = speed;
        m_bIntakeMotor->Set(speed);
    }
    void IntakeSubsystem::setAIntakeSpeed(double speed){
        aIntakeSpeed = speed;
        m_aIntakeMotor->Set(speed);
    }
    double IntakeSubsystem::getBIntakeSpeed(){
        return bIntakeSpeed;
    }
    double IntakeSubsystem::getAIntakeSpeed(){
        return aIntakeSpeed;
    }
    double IntakeSubsystem::getServoAngle(){
        return m_servo->GetAngle();
    }
    void IntakeSubsystem::setServoAngle(double angle){
        m_servo->SetAngle(angle);
    }
    
    