/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "components/SparkMaxMotorController.h"
#include <frc/smartdashboard/SmartDashboard.h>

SparkMaxMotorController::SparkMaxMotorController(int motorID, const std::string &name, rev::CANSparkMax::MotorType type): CommonMotorController(motorID, name){
    m_pMotor = new rev::CANSparkMax(motorID, type);
    m_pCurrentMode = (rev::ControlType)(-1);
    
}
void  SparkMaxMotorController::Config_kF(int slotIdx, double value, int timeoutMs){//Ignores slotIdx and timeoutMs
    m_pMotor->GetPIDController().SetFF(value);
}
void  SparkMaxMotorController::Config_kP(int slotIdx, double value, int timeoutMs){//Ignores slotIdx and timeoutMs
    m_pMotor->GetPIDController().SetP(value);
}
void  SparkMaxMotorController::Config_kI(int slotIdx, double value, int timeoutMs){//Ignores slotIdx and timeoutMs
    m_pMotor->GetPIDController().SetI(value);
}
void  SparkMaxMotorController::Config_kD(int slotIdx, double value, int timeoutMs){//Ignores slotIdx and timeoutMs
    m_pMotor->GetPIDController().SetD(value);
}
void  SparkMaxMotorController::Config_IntegralZone(int slotIdx, int izone, int timeoutMs) {
     m_pMotor->GetPIDController().SetIZone(izone);
}
void  SparkMaxMotorController::ConfigMaxIntegralAccumulator(int slotIdx, double iaccum, int timeoutMs) {
    m_pMotor->GetPIDController().SetIMaxAccum(iaccum);
}
void  SparkMaxMotorController::EnableVoltageCompensation(bool enable) {
    m_pMotor->EnableVoltageCompensation(enable);
}
void  SparkMaxMotorController::SetInverted(bool isInverted) {
    // m_pMotor->
    // frc::SmartDashboard::PutBoolean("motor inverted", isInverted);
    m_pMotor->SetInverted(isInverted);
}
void SparkMaxMotorController::Set(double speed){
    m_pMotor->Set(speed);
}
void SparkMaxMotorController::Set(CommonModes mode, double value){
    if(CommonModesToControlType(mode, m_pCurrentMode)){
        Set(value);
        frc::SmartDashboard::PutBoolean("PercentMode", true);
    }else{
        m_setpoint = value;
        frc::SmartDashboard::PutBoolean("PercentMode", false);
        m_pMotor->GetPIDController().SetReference(value, m_pCurrentMode);
    }
    frc::SmartDashboard::PutNumber("SparkMaxVel", value);
}
void SparkMaxMotorController:: Set(CommonModes mode, double demand0, DemandType demand1Type, double demand1){
    if(CommonModesToControlType(mode, m_pCurrentMode)){
        Set(demand0);
    }else{
        m_setpoint = demand0;
        m_pMotor->GetPIDController().SetReference(demand0, m_pCurrentMode);
    }
}
void SparkMaxMotorController::ConfigFactoryDefault(){
    m_pMotor->RestoreFactoryDefaults();
}
double SparkMaxMotorController::GetVelocity(){
    return m_pMotor->GetEncoder().GetVelocity();
}
void SparkMaxMotorController::SetEncoderPosition(double pos){
    m_pMotor->GetEncoder().SetPosition(pos);
}
void SparkMaxMotorController::SetVelocityConversionFactor(double factor){
    m_pMotor->GetEncoder().SetVelocityConversionFactor(factor);
}
double SparkMaxMotorController::GetClosedLoopError(){
    double temp  = 0.0;
    if(m_pCurrentMode == rev::ControlType::kVelocity){//velocity  controller
        temp = m_pMotor->GetEncoder().GetVelocity();
    }else if(m_pCurrentMode == rev::ControlType::kPosition){//position controller
        temp = m_pMotor->GetEncoder().GetPosition();
    }else{//percent controller
        temp = m_pMotor->Get();
    }
    return m_setpoint - temp;
}
void SparkMaxMotorController::SetNeutralMode(rev::CANSparkMax::IdleMode mode){
    m_pMotor->SetIdleMode(mode);
    
}
double SparkMaxMotorController::GetPos(){
    return m_pMotor->GetEncoder().GetPosition();
}
// void SparkMaxMotorController::ConfigFactoryDefault(){//TODO finish
//     m_pMotor->RestoreFactoryDefaults();
// }
bool SparkMaxMotorController::CommonModesToControlType(CommonModes mode, rev::ControlType& retMode){
    switch(mode)
    {
    case CommonModes::DutyCycle:      retMode = rev::ControlType::kDutyCycle; break;
    case CommonModes::Velocity:       retMode = rev::ControlType::kVelocity;break;
    case CommonModes::Voltage:        retMode = rev::ControlType::kVoltage;break;
    case CommonModes::Position:       retMode = rev::ControlType::kPosition;break;
    case CommonModes::SmartMotion:    retMode = rev::ControlType::kSmartMotion;break;
    case CommonModes::Current:        retMode = rev::ControlType::kVoltage;break;
    case CommonModes::SmartVelocity:  retMode = rev::ControlType::kSmartVelocity;break;
    case CommonModes::MotionMagic:    retMode = rev::ControlType::kSmartMotion;break;
    default: retMode = (rev::ControlType)(-1); return true;
    }

    return false;
}