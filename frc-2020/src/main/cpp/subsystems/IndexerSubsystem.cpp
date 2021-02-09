/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/IndexerSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

IndexerSubsystem::IndexerSubsystem() :
                m_indexerSpeed(0),
                m_indexerMotor(VictorIDs::kIndexerMotorID, "IndexMotor"),
                m_trenchBeamBreak(DigitalInputs::kTrenchBeamBreakPort),
                m_bottomBeamBreak(DigitalInputs::kBottomBeamBreakPort){
                    m_indexerMotor.ConfigFactoryDefault();
                    m_indexerMotor.SetInverted(true);
                    m_indexerMotor.SetNeutralMode(NeutralMode::Brake);
                }

// This method will be called once per scheduler run
void IndexerSubsystem::Periodic() {
    frc::SmartDashboard::PutBoolean("trench Beam", m_trenchBeamBreak.Get());
    frc::SmartDashboard::PutBoolean("bottom Beam", !m_bottomBeamBreak.Get());
    
}

void IndexerSubsystem::setIndexerSpeed(double speed){
        m_indexerSpeed = speed;
        m_indexerMotor.Set(speed);
    }

double IndexerSubsystem::getIndexerSpeed(){
        return m_indexerSpeed;
}
bool IndexerSubsystem::isBallInTrench(){
    return m_trenchBeamBreak.Get();
}
bool IndexerSubsystem::isBallInBottomBeamBreak(){
    return !m_bottomBeamBreak.Get();
}
