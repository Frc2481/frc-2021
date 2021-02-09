/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "components/TalonSRXMotorController.h"
#include <frc/smartdashboard/SmartDashboard.h>

TalonSRXMotorController::TalonSRXMotorController(int motorID, const std::string &name): CommonMotorController(motorID, name){
    m_pMotor = new TalonSRX(motorID);
}
void  TalonSRXMotorController::SetStatusFramePeriod(StatusFrameEnhanced frame, uint8_t periodMs, int timeoutMs){
    m_pMotor->SetStatusFramePeriod(frame, periodMs, timeoutMs);
}
void  TalonSRXMotorController::ConfigMotionAcceleration(int sensorUnitsPer100ms, int timeoutMs){
    m_pMotor->ConfigMotionAcceleration(sensorUnitsPer100ms, timeoutMs);
}
void  TalonSRXMotorController::ConfigMotionSCurveStrength(int curveStrength, int timeoutMs){
    m_pMotor->ConfigMotionSCurveStrength(curveStrength, timeoutMs);
}
void  TalonSRXMotorController::ConfigMotionCruiseVelocity(int sensorUnitsPer100ms, int timeoutMs){
    m_pMotor->ConfigMotionCruiseVelocity(sensorUnitsPer100ms, timeoutMs);
}
void  TalonSRXMotorController::Config_kF(int slotIdx, double value, int timeoutMs){
    m_pMotor->Config_kF(slotIdx, value, timeoutMs);
}

void  TalonSRXMotorController::config_PID(const TalonSRXPIDSetConfiguration& pid, int pidIdx, int timeoutMs){
    m_pMotor->ConfigurePID(pid,  pidIdx, timeoutMs);
}
void  TalonSRXMotorController::SelectProfileSlot(int slotIdx, int pidIdx){
    m_pMotor->SelectProfileSlot(0, 0);
}
void  TalonSRXMotorController::Config_kP(int slotIdx, double value, int timeoutMs){
    m_pMotor->Config_kP(slotIdx, value, timeoutMs);
}
void  TalonSRXMotorController::Config_kI(int slotIdx, double value, int timeoutMs){
    m_pMotor->Config_kI(slotIdx, value, timeoutMs);
}
void  TalonSRXMotorController::Config_kD(int slotIdx, double value, int timeoutMs){
    m_pMotor->Config_kD(slotIdx, value, timeoutMs);
}
void  TalonSRXMotorController::Config_IntegralZone(int slotIdx, int izone, int timeoutMs) {
     m_pMotor->Config_IntegralZone(slotIdx, izone, timeoutMs);
}
void  TalonSRXMotorController::ConfigMaxIntegralAccumulator(int slotIdx, double iaccum, int timeoutMs) {
    m_pMotor->ConfigMaxIntegralAccumulator (slotIdx, iaccum, timeoutMs);
}
void  TalonSRXMotorController::SetNeutralMode(NeutralMode neutralMode) {
    m_pMotor->SetNeutralMode(neutralMode);
}
void  TalonSRXMotorController::EnableVoltageCompensation(bool enable) {
    m_pMotor->EnableVoltageCompensation(enable);
}
void  TalonSRXMotorController::ConfigVoltageCompSaturation(double voltage, int timeoutMs) {
    m_pMotor->ConfigVoltageCompSaturation(voltage, timeoutMs);
}
void  TalonSRXMotorController::ConfigNeutralDeadband(double percentDeadband, int timeoutMs) {
    m_pMotor->ConfigNeutralDeadband(percentDeadband, timeoutMs);
}
void  TalonSRXMotorController::ConfigNominalOutputForward(double percentOut, int timeoutMs) {
    m_pMotor->ConfigNominalOutputForward(percentOut, timeoutMs);
}
void  TalonSRXMotorController::ConfigNominalOutputReverse(double percentOut, int timeoutMs) {
    m_pMotor->ConfigNominalOutputReverse(percentOut, timeoutMs);
}
void  TalonSRXMotorController::ConfigPeakOutputForward(double percentOut, int timeoutMs) {
    m_pMotor->ConfigPeakOutputForward(percentOut, timeoutMs);
}
void  TalonSRXMotorController::ConfigPeakOutputReverse(double percentOut, int timeoutMs) {
    m_pMotor->ConfigPeakOutputReverse(percentOut, timeoutMs);
}
void  TalonSRXMotorController::SetSensorPhase(bool PhaseSensor) {
    m_pMotor->SetSensorPhase(PhaseSensor);
}
void  TalonSRXMotorController::SetInverted(bool isInverted) {
    
    m_pMotor->SetInverted(isInverted);
}
void TalonSRXMotorController::Set(CommonModes mode, double value){
    
    m_pMotor->Set(CommonModeToControllMode(mode), value);
}
void TalonSRXMotorController::Set(double speed){
    m_pMotor->Set(ControlMode::PercentOutput, speed);
}
void TalonSRXMotorController::Set(CommonModes mode, double demand0, ctre::phoenix::motorcontrol::DemandType demand1Type, double demand1){
    m_pMotor->Set(CommonModeToControllMode(mode), demand0, demand1Type, demand1);
}
void TalonSRXMotorController::ConfigFactoryDefault(){
    m_pMotor->ConfigFactoryDefault();
}
void TalonSRXMotorController::ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice feedbackDevice, int pidIdx, int timeoutMs){
    m_pMotor->ConfigSelectedFeedbackSensor(feedbackDevice, pidIdx, timeoutMs);
}
int TalonSRXMotorController::GetSelectedSensorPosition(int id){
    return m_pMotor->GetSelectedSensorPosition(id);
}
int TalonSRXMotorController::isSensorConnected(){
    return m_pMotor->GetSensorCollection().GetPulseWidthRiseToRiseUs();
}
double TalonSRXMotorController::GetVelocity(){
    return m_pMotor->GetSelectedSensorVelocity()*m_factor;
}
void TalonSRXMotorController::SetEncoderPosition(double pos){
    m_pMotor->SetSelectedSensorPosition(pos);
}
void TalonSRXMotorController::SetVelocityConversionFactor(double factor){
    m_factor = factor;
}
double TalonSRXMotorController::GetClosedLoopError(){
    return m_pMotor->GetClosedLoopError();
}
ControlMode TalonSRXMotorController::CommonModeToControllMode(CommonModes mode){
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