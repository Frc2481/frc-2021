    //TODO Copied this from 2020 because intake supposedly similar, INCOMPLETE. -Sean H.

#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"
#include "RobotParameters.h"
#include "Constants.h"
    IntakeSubsystem::IntakeSubsystem(){
        m_servo = new frc::Servo(0);
        m_aIntakeMotor = new TalonSRXMotorController(TalonIDs::kAIntakeID, "Front Intake Motor");
        m_aIntakeMotor->ConfigFactoryDefault();
        m_bIntakeMotor = new TalonSRXMotorController(TalonIDs::kBIntakeID,"Left Intake Motor");
        m_bIntakeMotor->ConfigFactoryDefault();
    }

    void IntakeSubsystem::Periodic(){
    }
    void IntakeSubsystem::setBIntakeSpeed(double speed){
        m_bIntakeMotor->Set(speed);
    }
    void IntakeSubsystem::setAIntakeSpeed(double speed){
        m_aIntakeMotor->Set(speed);
    }

    void IntakeSubsystem::setAIntakeCurrent(double current){
        m_aIntakeMotor->Set(CommonModes::Current, current);
    }
    void IntakeSubsystem::setBIntakeCurrent(double current){
        m_bIntakeMotor->Set(CommonModes::Current, current);
    }
    double IntakeSubsystem::getIntakeACurrent(){
        return m_aIntakeMotor->GetCurrentOutput();
    }

    double IntakeSubsystem::getBIntakeSpeed(){
        return m_bIntakeMotor->GetVelocity();
    }
    double IntakeSubsystem::getAIntakeSpeed(){
        return m_aIntakeMotor->GetVelocity();
    }
    double IntakeSubsystem::getServoAngle(){
        return m_servo->GetAngle();
    }
    void IntakeSubsystem::setServoAngle(double angle){
        m_servo->SetAngle(angle);
    }
    
    