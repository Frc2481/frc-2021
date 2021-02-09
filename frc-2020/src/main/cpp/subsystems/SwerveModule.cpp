/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/SwerveModule.h"
#include "components/CTREMagEncoder.h"
#include <frc/geometry/Rotation2d.h>
#include <wpi/math>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"
#include "components/MotorPositionController.h"
#include "Utils/MathConstants.h"
#include "Utils/NormalizeToRange.h"
/*
void SetTalonConfig(TalonConfig config) {
  m_motor->GetPIDController()->SetP(talonConfig.slo0.kp);
  m_motor->GetPIDController()->SetI(talonConfig.slo0.ki);
  m_motor->GetPIDController()->SetD(talonConfig.slo0.kd);
}
*/


SwerveModule::SwerveModule(int driveMotorID, int turningMotorID,
                           bool driveEncoderReversed,
                           bool turningEncoderReversed, const std::string &name) :m_reverseDriveEncoder(driveEncoderReversed),
      m_reverseTurningEncoder(turningEncoderReversed),  
      m_name(name){
      m_driveMotor = new TalonFXMotorController(driveMotorID, name);
      m_turningMotor = new TalonSRXMotorController(turningMotorID, m_name);
      m_turningMotor->ConfigFactoryDefault();
      m_turningEncoder = new CTREMagEncoder(m_turningMotor, name);
      m_driveMotor->SetVelocityConversionFactor(RobotParameters::k_driveMotorEncoderTicksToMPS); // (1 rev / 5 v) * .16 m/rev
      m_driveMotor->ConfigFactoryDefault();
      m_driveMotor->SetInverted(driveEncoderReversed);
      m_driveMotor->Config_kP(0, 0.1);//.07
      m_driveMotor->Config_kI(0, 0);
      m_driveMotor->Config_kD(0, 0);//.035
      m_driveMotor->Config_kF(0, 1023/(RobotParameters::k_maxSpeed/RobotParameters::k_driveMotorEncoderTicksToMPS));
      m_driveMotor->Config_IntegralZone(0, 0);
      m_driveMotor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  // // Set the distance per pulse for the drive encoder. We can simply use the
  // // distance traveled for one rotation of the wheel divided by the encoder
  // // resolution.
  // m_driveEncoder.SetDistancePerPulse(
  //     ModuleConstants::kDriveEncoderDistancePerPulse);

  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * wpi::math::pi)
  // divided by the encoder resolution.

  // m_turningEncoder.SetDistancePerPulse(
  //     ModuleConstants::kTurningEncoderDistancePerPulse);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(units::radian_t(-wpi::math::pi),
                                               units::radian_t(wpi::math::pi));
  
	

	m_turningMotorController = new MotorPositionController(
		m_turningMotor,
		m_turningEncoder,
		false,
		m_reverseTurningEncoder,
		RobotParameters::k_steerMotorControllerKp,
		RobotParameters::k_steerMotorControllerKi,
		RobotParameters::k_steerMotorControllerKd,
		RobotParameters::k_steerMotorControllerKv,
		RobotParameters::k_steerMotorControllerKap,
		RobotParameters::k_steerMotorControllerKan,
		RobotParameters::k_steerMotorControllerKsf,
		0,
		0,
		RobotParameters::k_ctreMagEncoderTicksPerRev * RobotParameters::k_steerEncoderToWheelGearRatio);
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{m_driveMotor->GetVelocity()},//MATH_CONSTANTS_PI
          frc::Rotation2d(units::degree_t(m_turningEncoder->getAngle()))};
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState& state, bool percentMode) {
  m_turningEncoder->update();
  
  // Calculate the drive output from the drive PID controller.
  // const auto driveOutput = m_drivePIDController.Calculate(
  //     m_driveMotor->GetEncoder().GetVelocity(), state.speed.to<double>());

  float currentAngle = units::degree_t(m_turningEncoder->getAngle()).to<double>();
  float driveMotorRPM = state.speed.to<double>();///RobotParameters::k_driveMotorEncoderRPMToMPS;
  float desiredAngle = state.angle.Degrees().to<double>();

  // // Calculate the turning motor output from the turning PID controller.
  // auto turnOutput = m_turningPIDController.Calculate(
  //     units::radian_t(units::degree_t(m_turningEncoder->getAngle())), state.angle.Radians());
    
  //   frc::SmartDashboard::PutNumber("Desired", state.angle.Degrees().to<double>());
  //   frc::SmartDashboard::PutNumber("Current", units::degree_t(m_turningEncoder->getAngle()).to<double>());
  //   frc::SmartDashboard::PutNumber("Turn Output", turnOutput); //TODO: These shouldn't stay here
  //   frc::SmartDashboard::PutNumber("Speed", m_driveMotor->GetEncoder().GetVelocity());
      // printf("%s: current angle %f desired angle %f\n", 
      //     m_name.c_str(), 
      //     units::radian_t(m_turningEncoder->getAngle()).to<double>(), 
      //     state.angle.Radians().to<double>());
  // printf("\nC A: %f\n", currentAngle); //TODO: Remove these printfs once turn issue fixed
  // printf("\nD A: %f\n", desiredAngle);
frc::SmartDashboard::PutNumber("driveMotorVelpreLogic",driveMotorRPM);
frc::SmartDashboard::PutNumber(m_name, m_turningEncoder->getAngle());
frc::SmartDashboard::PutNumber(m_name + " drive", m_driveMotor->GetVelocity());
  if(fabs(normalizeToRange::RangedDifference(currentAngle - desiredAngle, -180, 180)) > 90){//used to be 90
    desiredAngle = normalizeToRange::NormalizeToRange(desiredAngle+180, -180, 180, true);
    driveMotorRPM = driveMotorRPM * -1;
  }
  if(fabs(driveMotorRPM) < 0.01){//TODO find better zone
    desiredAngle = currentAngle;
    // driveMotorRPM= 0;
    // printf("\nD%f\n", driveMotorRPM);
  }
frc::SmartDashboard::PutNumber(m_name + " desiredAngle", desiredAngle);
  // printf("\nUpdated Current Angle: %f\n", currentAngle);
  // printf("\nUpdated Desired Angle: %f\n", desiredAngle);
  // printf("\nUpdated Desired Speed: %f\n", driveMotorRPM);

  // Set the motor outputs.
  if(fabs((m_driveMotor->GetVelocity())  <= RobotParameters::k_driveWheelSlotError && driveMotorRPM == 0.0) || percentMode){
    m_driveMotor->Set(driveMotorRPM); 
  }else{
    m_driveMotor->Set(CommonModes::Velocity, driveMotorRPM);
  }
  frc::SmartDashboard::PutNumber("driveMotorVelfinal",driveMotorRPM);
  // m_turningMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, turnOutput);

  m_turningMotorController->updateAngular(desiredAngle, 0, 0);
}

void SwerveModule::ResetEncoders() {
  m_driveMotor->SetEncoderPosition(0);
  m_turningEncoder->zero();

}

void SwerveModule::updateSteerPID(double p, double i, double d){
  // m_turningPIDController.SetPID(p, i, d);
  // printf("Steer P: %0.1f, I: %0.1f, D: %0.1f", p, i, d);
  m_turningPIDController.SetP(p);
  m_turningPIDController.SetI(i);
  m_turningPIDController.SetD(d);
}

void SwerveModule::updateDrivePID(double p, double i, double d, double f){
  // printf("Drive P: %0.1f, I: %0.1f, D: %0.1f", p, i, d);
  // m_drivePIDController.SetPID(p, i, d);
  m_driveMotor->Config_kP(0,p);
  m_driveMotor->Config_kI(0,i);
  m_driveMotor->Config_kD(0,d);
  m_driveMotor->Config_kF(0,f);
}
void SwerveModule::setCoast(){
  m_driveMotor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
}

void SwerveModule::setBrake(){
  m_driveMotor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

void SwerveModule::DriveArc(double arcLength, double wheelAngle){
  m_turningMotorController->updateAngular(wheelAngle, 0, 0);
  m_driveMotor->ConfigMotionCruiseVelocity((RobotParameters::k_maxSpeed)/RobotParameters::k_driveMotorEncoderTicksToMPS);
  m_driveMotor->ConfigMotionAcceleration(((RobotParameters::k_maxSpeed)/RobotParameters::k_driveMotorEncoderTicksToMPS)*2);
  m_driveMotor->Set(CommonModes::MotionMagic, m_driveMotor->GetPos() + arcLength/RobotParameters::k_driveMotorEncoderTicksToMeters);
}
