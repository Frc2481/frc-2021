/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/FeederSubsystem.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "RobotParameters.h"
FeederSubsystem::FeederSubsystem() :
        m_feeder(FalconIDs::kFeederMotorID, "FeederMotor"),   
        m_midBeamBreak(DigitalInputs::kMidBeamBreakPort),
        m_topBeamBreak(DigitalInputs::kTopBeamBreakPort){
            m_feeder.ConfigFactoryDefault();
            m_feeder.SetNeutralMode(NeutralMode::Brake);
            m_feeder.Config_kP(0,RobotParameters::k_feederP);//15
            m_feeder.Config_kI(0,0);
            m_feeder.Config_kD(0,RobotParameters::k_feederD);
            // m_feeder.SetEncoderPosition(0);
            m_feeder.SetSensorPhase(true);
            m_feeder.SetInverted(true);
            m_manualControl = false;
            m_ballCount = 0;
            setSensorPos(FeederConstants::kIntakeBallDistance +1);
        }


// This method will be called once per scheduler run
void FeederSubsystem::Periodic(){
    m_midBeamState = !m_midBeamBreak.Get();
    m_topBeamState = !m_topBeamBreak.Get();
    frc::SmartDashboard::PutBoolean("mid beam", m_midBeamState);
    frc::SmartDashboard::PutBoolean("top beam", m_topBeamState);
    frc::SmartDashboard::PutNumber("ball count", m_ballCount);
    frc::SmartDashboard::PutNumber("feeder pos", getFeederPos());
    if(m_midBeamState && !m_midBeamPrevState){
        m_ballCount++;
        zeroPos();
        // printf("zero pos ball in mid prev was not\n");
    }
    if(!m_topBeamState && m_topBeamPrevState){
         m_ballCount--;
    }
    if(m_ballCount < 0){
        m_ballCount = 0;
    }
    m_midBeamPrevState = m_midBeamState;
    m_topBeamPrevState = m_topBeamState;
}

void FeederSubsystem::setFeederSpeed(double speed){
    m_feederSpeed = speed;
    m_manualControl = true;
    m_feeder.Set(speed);
}
double FeederSubsystem::getFeederPos(){
    return m_feeder.GetPos()/2048.0;
}
int FeederSubsystem::getBallCount(){
        return m_ballCount;
}
bool FeederSubsystem::isBallInTopBeamBreak(){
    return m_topBeamState;
}
bool FeederSubsystem::isBallInMidBeamBreak(){
    return m_midBeamState;
}
void FeederSubsystem::setFeederPos(double pos){
    // printf("set feeder pos: %f\n", pos);
    m_feederSpeed = 0;
    m_manualControl = false;
    m_feeder.Set(CommonModes::Position, pos*2048);
}
bool FeederSubsystem::getPrevBottomState(){
    return m_midBeamPrevState;
}
void FeederSubsystem::zeroPos(){
    // printf("zero feeder pos\n");
    m_feeder.SetEncoderPosition(0);
}
void FeederSubsystem::zeroBallCount(){
    m_ballCount = 0;
}

void FeederSubsystem::setBallCount(int count){
    m_ballCount = count;
}

bool FeederSubsystem::isFeederReady(){
    return (!m_topBeamState && getFeederPos() + FeederConstants::kBallDistanceTolerance >= FeederConstants::kIntakeBallDistance) //normal while intaking
            || (m_manualControl && getFeederPos() + FeederConstants::kBallDistanceTolerance >= frc::SmartDashboard::GetNumber("feeder last ball distance", FeederConstants::kLastBallDistance));//while shooting mode
}
bool FeederSubsystem::isMidPrevBeam(){
    return m_midBeamPrevState;
}
double FeederSubsystem::getFeederSetSpeed(){
    return m_feederSpeed;
}
void FeederSubsystem::tunePID(double p,double i,double d,double f,double izone){
    m_feeder.Config_kP(0,p);
    m_feeder.Config_kI(0,i);
    m_feeder.Config_kD(0,d);
    m_feeder.Config_kF(0,f);
    m_feeder.Config_IntegralZone(0,izone);
}

void FeederSubsystem::setSensorPos(double pos){
    m_feeder.SetEncoderPosition(pos*2048);
    // printf("setting feeder pos to %f", pos);
}