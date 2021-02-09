    //TODO Copied this from 2020 because intake supposedly similar, INCOMPLETE. -Sean H.

#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"
#include "RobotParameters.h"

    IntakeSubsystem::IntakeSubsystem() :
    leftSpeed(0.0),
    rightSpeed(0.0)
    {
        m_beamBreak = new frc::DigitalInput(BeamBreaks::kBeamBreakID);
        m_rightMotor = new VictorMotorController(VictorIDs::kRightIntakeID, "Left Intake Motor");
        m_leftMotor = new VictorMotorController(VictorIDs::kLeftIntakeID,"Right Intake Motor");
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
    double IntakeSubsystem::getBeamBreak(){
        return m_beamBreak->Get();
    }

    
    