/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "components/TalonFXMotorController.h"
#include "RobotParameters.h"

TalonFXMotorController::TalonFXMotorController(int motorID, const std::string &name) : CommonMotorController(motorID, name) {
    m_pMotor = new TalonFX(motorID);
}
void  TalonFXMotorController::SetStatusFramePeriod(StatusFrameEnhanced frame, uint8_t periodMs, int timeoutMs){
    m_pMotor->SetStatusFramePeriod(frame, periodMs, timeoutMs);
}
void  TalonFXMotorController::ConfigMotionAcceleration(int sensorUnitsPer100ms, int timeoutMs){
    m_pMotor->ConfigMotionAcceleration(sensorUnitsPer100ms, timeoutMs);
}
void  TalonFXMotorController::ConfigMotionSCurveStrength(int curveStrength, int timeoutMs){
    m_pMotor->ConfigMotionSCurveStrength(curveStrength, timeoutMs);
}
void  TalonFXMotorController::ConfigMotionCruiseVelocity(int sensorUnitsPer100ms, int timeoutMs){
    m_pMotor->ConfigMotionCruiseVelocity(sensorUnitsPer100ms, timeoutMs);
}
void  TalonFXMotorController::Config_kF(int slotIdx, double value, int timeoutMs){
    m_pMotor->Config_kF(slotIdx, value, timeoutMs);
}
void  TalonFXMotorController::config_PID(const TalonFXPIDSetConfiguration& pid, int pidIdx, int timeoutMs){
    m_pMotor->ConfigurePID(pid,  pidIdx, timeoutMs);
}
void  TalonFXMotorController::SelectProfileSlot(int slotIdx, int pidIdx){
    m_pMotor->SelectProfileSlot(0, 0);
}
void  TalonFXMotorController::Config_kP(int slotIdx, double value, int timeoutMs){
    m_pMotor->Config_kP(slotIdx, value, timeoutMs);
}
void  TalonFXMotorController::Config_kI(int slotIdx, double value, int timeoutMs){
    m_pMotor->Config_kI(slotIdx, value, timeoutMs);
}
void  TalonFXMotorController::Config_kD(int slotIdx, double value, int timeoutMs){
    m_pMotor->Config_kD(slotIdx, value, timeoutMs);
}
void  TalonFXMotorController::Config_IntegralZone(int slotIdx, int izone, int timeoutMs) {
     m_pMotor->Config_IntegralZone(slotIdx, izone, timeoutMs);
}
void  TalonFXMotorController::ConfigMaxIntegralAccumulator(int slotIdx, double iaccum, int timeoutMs) {
    m_pMotor->ConfigMaxIntegralAccumulator (slotIdx, iaccum, timeoutMs);
}
void  TalonFXMotorController::SetNeutralMode(NeutralMode neutralMode) {
    m_pMotor->SetNeutralMode(neutralMode);
}
void  TalonFXMotorController::EnableVoltageCompensation(bool enable) {
    m_pMotor->EnableVoltageCompensation(enable);
}
void  TalonFXMotorController::ConfigVoltageCompSaturation(double voltage, int timeoutMs) {
    m_pMotor->ConfigVoltageCompSaturation(voltage, timeoutMs);
}
void  TalonFXMotorController::ConfigNeutralDeadband(double percentDeadband, int timeoutMs) {
    m_pMotor->ConfigNeutralDeadband(percentDeadband, timeoutMs);
}
void  TalonFXMotorController::ConfigNominalOutputForward(double percentOut, int timeoutMs) {
    m_pMotor->ConfigNominalOutputForward(percentOut, timeoutMs);
}
void  TalonFXMotorController::ConfigNominalOutputReverse(double percentOut, int timeoutMs) {
    m_pMotor->ConfigNominalOutputReverse(percentOut, timeoutMs);
}
void  TalonFXMotorController::ConfigPeakOutputForward(double percentOut, int timeoutMs) {
    m_pMotor->ConfigPeakOutputForward(percentOut, timeoutMs);
}
void  TalonFXMotorController::ConfigPeakOutputReverse(double percentOut, int timeoutMs) {
    m_pMotor->ConfigPeakOutputReverse(percentOut, timeoutMs);
}
void  TalonFXMotorController::SetSensorPhase(bool PhaseSensor) {
    m_pMotor->SetSensorPhase(PhaseSensor);
}
void  TalonFXMotorController::SetInverted(bool isInverted) {
    m_pMotor->SetInverted(isInverted);
}
void TalonFXMotorController::Set(CommonModes mode, double value){
    if(CommonModes::Velocity == mode){
        m_pMotor->Set(FalconModeToCommonMode(mode),value* (1/m_factor));
    }else{
        m_pMotor->Set(FalconModeToCommonMode(mode), value);
    }
    
}
void TalonFXMotorController::Set(CommonModes mode, double demand0, DemandType demand1Type, double demand1){
    
    m_pMotor->Set(FalconModeToCommonMode(mode), demand0, demand1Type, demand1);
}
void TalonFXMotorController::Set(double speed){
    m_pMotor->Set(ControlMode::PercentOutput, speed);
}
void TalonFXMotorController::ConfigFactoryDefault(){
    m_pMotor->ConfigFactoryDefault();
}
void TalonFXMotorController::ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice feedbackDevice, int pidIdx, int timeoutMs){
    m_pMotor->ConfigSelectedFeedbackSensor(feedbackDevice, pidIdx, timeoutMs);
}
int TalonFXMotorController::GetSelectedSensorPosition(int id){
    // printf("+++++++++++getting pos+++++++++++\n");
    return m_pMotor->GetSelectedSensorPosition(id);
}
int TalonFXMotorController::isSensorConnected(){
    printf("Warning: GetSensorCollection() doesn't work with TalonFX, instead use TalonSRX\n");
    return 1; 
}
double TalonFXMotorController::GetVelocity(){
    return m_pMotor->GetSelectedSensorVelocity()*m_factor;
}
void TalonFXMotorController::SetEncoderPosition(double pos){
    m_pMotor->SetSelectedSensorPosition(pos);
}
void TalonFXMotorController::SetVelocityConversionFactor(double factor){
    // m_pMotor->ConfigSelectedFeedbackCoefficient(factor);
    m_factor = factor;
}
double TalonFXMotorController::GetClosedLoopError(){
    return m_pMotor->GetClosedLoopError();
}
TalonFX* TalonFXMotorController::GetBase(){
    return m_pMotor;
}
void TalonFXMotorController::Follow(TalonFX* motor){
    m_pMotor->Follow(*motor);
}
double TalonFXMotorController::GetPos(){
    return m_pMotor->GetSelectedSensorPosition();
}
// void TalonFXMotorController::SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode mode){
//     m_pMotor->SetNeutralMode(mode);
// }
ControlMode TalonFXMotorController::FalconModeToCommonMode(CommonModes mode){
    switch(mode)
    {
    case CommonModes::Velocity:         return ControlMode::Velocity;
    case CommonModes::Voltage:          return ControlMode::Current;
    case CommonModes::Position:         return ControlMode::Position;
    case CommonModes::SmartMotion:      return ControlMode::MotionMagic;
    case CommonModes::Current:          return ControlMode::Current;
    case CommonModes::SmartVelocity:    return ControlMode::Velocity;
    case CommonModes::PercentOutput:    return ControlMode::PercentOutput;
    case CommonModes::Follower:         return ControlMode::Follower;
    case CommonModes::MotionProfile:    return ControlMode::MotionProfile;
    case CommonModes::MotionMagic:      return ControlMode::MotionMagic;
    case CommonModes::MotionProfileArc: return ControlMode::MotionProfileArc;
    case CommonModes::Disabled:         return ControlMode::Disabled;
    default: return ControlMode::PercentOutput;
    }
}