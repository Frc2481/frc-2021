/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ClimberSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
ClimberSubsystem::ClimberSubsystem() : m_rotateToHeight(0), m_climber(FalconIDs::kClimberID, "Climber Motor") {
    m_climber.SetNeutralMode(NeutralMode::Brake);
    m_climber.Config_kP(0,RobotParameters::k_climbP);
    m_climber.Config_kI(0,RobotParameters::k_climbI);
    m_climber.Config_kD(0,RobotParameters::k_climbD);
    m_climber.Config_kF(0, RobotParameters::k_climbF);
    m_climber.SetEncoderPosition(0);
    m_climber.SetInverted(true);
}
// -433583.0
// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {
    // m_climber.GetVelocity();
    if((m_climber.GetSelectedSensorPosition()  >=  m_climbPos - 1000) //normal while intaking
            && m_climber.GetSelectedSensorPosition()  <=  m_climbPos + 1000){
                m_climber.Set(0);
                m_climberReady = true;

            }
    // if(fabs(m_climber.GetSelectedSensorPosition() - frc::SmartDashboard::GetNumber("climberSetPos", 100) )>500)
    frc::SmartDashboard::PutNumber("climb pos",m_climber.GetSelectedSensorPosition());
}

void ClimberSubsystem::climb(double percent){
    m_climber.Set(CommonModes::PercentOutput, percent);
}
void ClimberSubsystem::stop(){
    m_climber.Set(0);
}
void ClimberSubsystem::grabBar(){
    m_climber.Set(CommonModes::Position, ClimberConstants::kClimbingHeight);
}
void ClimberSubsystem::zeroPos(){
    m_climber.SetEncoderPosition(0);
    m_climbPos = 0;
}
double ClimberSubsystem::getPos(){
    return m_climber.GetSelectedSensorPosition();
}
void ClimberSubsystem::goToPos(double pos){
    m_climbPos = pos;
    m_climber.Set(CommonModes::Position, pos);
    m_climberReady = false;
}
void ClimberSubsystem::pid(double p, double i, double d, double f, double izone){
    m_climber.Config_kP(0,p);
    m_climber.Config_kI(0, i);
    m_climber.Config_kD(0,d);
    m_climber.Config_IntegralZone(0,izone);
    m_climber.Config_kF(0, f);
}
bool ClimberSubsystem::climberReady(){
    return m_climberReady;
}