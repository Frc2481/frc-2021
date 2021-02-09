/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "components/VictorMotorController.h"
#include <frc/smartdashboard/SmartDashboard.h>

VictorMotorController::VictorMotorController(int motorID, const std::string &name): CommonMotorController(motorID, name){
    m_pMotor = new VictorSPX(motorID);
}
void  VictorMotorController::SetStatusFramePeriod(StatusFrameEnhanced frame, uint8_t periodMs, int timeoutMs){
    m_pMotor->SetStatusFramePeriod(frame, periodMs, timeoutMs);
}
void  VictorMotorController::ConfigMotionAcceleration(int sensorUnitsPer100ms, int timeoutMs){
    m_pMotor->ConfigMotionAcceleration(sensorUnitsPer100ms, timeoutMs);
}
void  VictorMotorController::ConfigMotionSCurveStrength(int curveStrength, int timeoutMs){
    m_pMotor->ConfigMotionSCurveStrength(curveStrength, timeoutMs);
}
void  VictorMotorController::ConfigMotionCruiseVelocity(int sensorUnitsPer100ms, int timeoutMs){
    m_pMotor->ConfigMotionCruiseVelocity(sensorUnitsPer100ms, timeoutMs);
}
void  VictorMotorController::Config_kF(int slotIdx, double value, int timeoutMs){
    m_pMotor->Config_kF(slotIdx, value, timeoutMs);
}

// void  VictorMotorController::config_PID(const VictorSPXPIDSetConfiguration& pid, int pidIdx, int timeoutMs, bool optimize){
//     // m_pMotor->Co(pid,  pidIdx, timeoutMs, optimize);//TODO find
// }
void  VictorMotorController::SelectProfileSlot(int slotIdx, int pidIdx){
    m_pMotor->SelectProfileSlot(0, 0);
}
void  VictorMotorController::Config_kP(int slotIdx, double value, int timeoutMs){
    m_pMotor->Config_kP(slotIdx, value, timeoutMs);
}
void  VictorMotorController::Config_kI(int slotIdx, double value, int timeoutMs){
    m_pMotor->Config_kI(slotIdx, value, timeoutMs);
}
void  VictorMotorController::Config_kD(int slotIdx, double value, int timeoutMs){
    m_pMotor->Config_kD(slotIdx, value, timeoutMs);
}
void  VictorMotorController::Config_IntegralZone(int slotIdx, int izone, int timeoutMs) {
     m_pMotor->Config_IntegralZone(slotIdx, izone, timeoutMs);
}
void  VictorMotorController::ConfigMaxIntegralAccumulator(int slotIdx, double iaccum, int timeoutMs) {
    m_pMotor->ConfigMaxIntegralAccumulator (slotIdx, iaccum, timeoutMs);
}
void  VictorMotorController::SetNeutralMode(NeutralMode neutralMode) {
    m_pMotor->SetNeutralMode(neutralMode);
}
void  VictorMotorController::EnableVoltageCompensation(bool enable) {
    m_pMotor->EnableVoltageCompensation(enable);
}
void  VictorMotorController::ConfigVoltageCompSaturation(double voltage, int timeoutMs) {
    m_pMotor->ConfigVoltageCompSaturation(voltage, timeoutMs);
}
void  VictorMotorController::ConfigNeutralDeadband(double percentDeadband, int timeoutMs) {
    m_pMotor->ConfigNeutralDeadband(percentDeadband, timeoutMs);
}
void  VictorMotorController::ConfigNominalOutputForward(double percentOut, int timeoutMs) {
    m_pMotor->ConfigNominalOutputForward(percentOut, timeoutMs);
}
void  VictorMotorController::ConfigNominalOutputReverse(double percentOut, int timeoutMs) {
    m_pMotor->ConfigNominalOutputReverse(percentOut, timeoutMs);
}
void  VictorMotorController::ConfigPeakOutputForward(double percentOut, int timeoutMs) {
    m_pMotor->ConfigPeakOutputForward(percentOut, timeoutMs);
}
void  VictorMotorController::ConfigPeakOutputReverse(double percentOut, int timeoutMs) {
    m_pMotor->ConfigPeakOutputReverse(percentOut, timeoutMs);
}
void  VictorMotorController::SetSensorPhase(bool PhaseSensor) {
    m_pMotor->SetSensorPhase(PhaseSensor);
}
void  VictorMotorController::SetInverted(bool isInverted) {
    
    m_pMotor->SetInverted(isInverted);
}
void VictorMotorController::Set(CommonModes mode, double value){
    
    m_pMotor->Set(CommonModeToControllMode(mode), value);
}
void VictorMotorController::Set(double speed){
    m_pMotor->Set(ControlMode::PercentOutput, speed);
}
void VictorMotorController::Set(CommonModes mode, double demand0, ctre::phoenix::motorcontrol::DemandType demand1Type, double demand1){
    m_pMotor->Set(CommonModeToControllMode(mode), demand0, demand1Type, demand1);
}
void VictorMotorController::ConfigFactoryDefault(){
    m_pMotor->ConfigFactoryDefault();
}
void VictorMotorController::ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice feedbackDevice, int pidIdx, int timeoutMs){
    m_pMotor->ConfigSelectedFeedbackSensor(feedbackDevice, pidIdx, timeoutMs);
}
int VictorMotorController::GetSelectedSensorPosition(int id){
    return m_pMotor->GetSelectedSensorPosition(id);
}
void VictorMotorController::SetEncoderPosition(double pos){
    m_pMotor->SetSelectedSensorPosition(pos);
}
void VictorMotorController::SetVelocityConversionFactor(double factor){
    m_factor = factor;
}
double VictorMotorController::GetClosedLoopError(){
    return m_pMotor->GetClosedLoopError();
}
ControlMode VictorMotorController::CommonModeToControllMode(CommonModes mode){
    switch(mode)
    {
    case CommonModes::Velocity:    return ControlMode::Velocity;
    case CommonModes::Voltage:     return ControlMode::Current;
    case CommonModes::Position:      return ControlMode::Position;
    case CommonModes::SmartMotion:       return ControlMode::MotionMagic;
    case CommonModes::Current:        return ControlMode::Current;
    case CommonModes::SmartVelocity:      return ControlMode::Velocity;
    case CommonModes::PercentOutput:      return ControlMode::PercentOutput;
    case CommonModes::Follower:      return ControlMode::Follower;
    case CommonModes::MotionProfile:      return ControlMode::MotionProfile;
    case CommonModes::MotionMagic:      return ControlMode::MotionMagic;
    case CommonModes::MotionProfileArc:      return ControlMode::MotionProfileArc;
    case CommonModes::Disabled:      return ControlMode::Disabled;
    default: return ControlMode::PercentOutput;
    }
}