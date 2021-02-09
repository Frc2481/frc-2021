/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "components/CommonMotorController.h"
#pragma once

class TalonSRXMotorController : public CommonMotorController {
 
 public:
  TalonSRXMotorController(int motorID, const std::string &name);
  void  SetStatusFramePeriod(StatusFrameEnhanced frame, uint8_t periodMs, int timeoutMs = 0);
  void  ConfigMotionAcceleration(int sensorUnitsPer100ms, int timeoutMs = 0);
  void  ConfigMotionSCurveStrength(int curveStrength, int timeoutMs = 0);
  void  ConfigMotionCruiseVelocity(int sensorUnitsPer100ms, int timeoutMs = 0);
  void  Config_kF(int slotIdx, double value, int timeoutMs = 0);
  void  config_PID(const TalonSRXPIDSetConfiguration& pid, int pidIdx = 0, int timeoutMs = 50);
  void  SelectProfileSlot(int slotIdx, int pidIdx);
	void  Config_kP(int slotIdx, double value, int timeoutMs = 0);
	void  Config_kI(int slotIdx, double value, int timeoutMs = 0);
  void  Config_kD(int slotIdx, double value, int timeoutMs = 0);
  void  Config_IntegralZone(int slotIdx, int izone, int timeoutMs = 0);
  void  ConfigMaxIntegralAccumulator(int slotIdx, double iaccum, int timeoutMs = 0);
  void  SetNeutralMode(NeutralMode neutralMode);
  void  EnableVoltageCompensation(bool enable);
  void  ConfigVoltageCompSaturation(double voltage, int timeoutMs = 0);
  void  ConfigNeutralDeadband(double percentDeadband, int timeoutMs = 0);
  void  ConfigNominalOutputForward(double percentOut, int timeoutMs = 0);
  void  ConfigNominalOutputReverse(double percentOut, int timeoutMs = 0);
  void  ConfigPeakOutputForward(double percentOut, int timeoutMs = 0);
  void  ConfigPeakOutputReverse(double percentOut, int timeoutMs = 0);
  void  SetSensorPhase(bool PhaseSensor);
  void  SetInverted(bool isInverted);
  void Set(double speed);
  void Set(CommonModes mode, double value);
  void Set(CommonModes mode, double demand0, ctre::phoenix::motorcontrol::DemandType demand1Type, double demand1);
  void ConfigFactoryDefault();
  void ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice feedbackDevice, int pidIdx, int timeoutMs);
  int GetSelectedSensorPosition(int id);
  int isSensorConnected();
  double GetVelocity();
  void SetEncoderPosition(double pos);
  void SetVelocityConversionFactor(double factor);
  double GetClosedLoopError();
  ControlMode CommonModeToControllMode(CommonModes mode);
private:
  TalonSRX* m_pMotor;
  double m_factor = 1.0;
};
