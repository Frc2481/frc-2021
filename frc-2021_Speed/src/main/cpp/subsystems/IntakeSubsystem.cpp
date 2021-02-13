    //TODO Copied this from 2020 because intake supposedly similar, INCOMPLETE. -Sean H.

#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"
#include "RobotParameters.h"
#include "Constants.h"
    IntakeSubsystem::IntakeSubsystem() :
    leftSpeed(0.0),
    rightSpeed(0.0)
    {
        m_servo = new frc::Servo(0);
        m_rightMotor = new VictorMotorController(VictorIDs::kFrontIntakeID, "Front Intake Motor");
        m_rightMotor->ConfigFactoryDefault();
        m_leftMotor = new VictorMotorController(VictorIDs::kLeftIntakeID,"Left Intake Motor");
    }

    void IntakeSubsystem::Periodic(){
    }
    void IntakeSubsystem::setLeftSpeed(double speed){
        leftSpeed = speed;
        m_leftMotor->Set(speed);
    }
    void IntakeSubsystem::setRightSpeed(double speed){
        rightSpeed = speed;
        m_rightMotor->Set(speed);
    }
    double IntakeSubsystem::getLeftSpeed(){
        return leftSpeed;
    }
    double IntakeSubsystem::getRightSpeed(){
        return rightSpeed;
    }
    void IntakeSubsystem::setRightCurrent(double amp){
        m_rightMotor->Set(CommonModes::Current, amp);
    }

    double IntakeSubsystem::getServoAngle(){
        return m_servo->GetAngle();
    }
    void IntakeSubsystem::setServoAngle(double angle){
        m_servo->SetAngle(angle);
    }
    
    