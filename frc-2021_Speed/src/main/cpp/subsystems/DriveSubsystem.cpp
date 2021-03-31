/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/velocity.h>
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "Utils/NormalizeToRange.h"
#include <iostream>
#include <fstream>



using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_frontLeft{FalconIDs::kFrontLeftDriveMotorID,
                  TalonIDs::kFrontLeftTurningMotorID,
                  kFrontLeftDriveEncoderReversed,
                  kFrontLeftTurningEncoderReversed,
                  kFrontLeftTurningMotorReversed,
                  "FL_STEER_MOTOR_ENCODER"},

      m_backLeft{
                  FalconIDs::kBackLeftDriveMotorID,       
                  TalonIDs::kBackLeftTurningMotorID,
                  kBackLeftDriveEncoderReversed, 
                  kBackLeftTurningEncoderReversed,
                  kBackLeftTurningMotorReversed,
                  "BL_STEER_MOTOR_ENCODER"},

      m_frontRight{
                  FalconIDs::kFrontRightDriveMotorID,       
                  TalonIDs::kFrontRightTurningMotorID,
                  kFrontRightDriveEncoderReversed, 
                  kFrontRightTurningEncoderReversed,
                  kFrontRightTurningMotorReversed,
                  "FR_STEER_MOTOR_ENCODER"},

      m_backRight{
                  FalconIDs::kBackRightDriveMotorID,       
                  TalonIDs::kBackRightTurningMotorID,
                  kBackRightDriveEncoderReversed, 
                  kBackRightTurningEncoderReversed,
                  kBackRightTurningMotorReversed,
                  "BR_STEER_MOTOR_ENCODER"},

      m_odometry{kDriveKinematics,
                 frc::Rotation2d(units::degree_t(0)),
                 frc::Pose2d()},
      m_pChassisIMU{frc::SPI::kMXP}{
      setBrake();
      }


void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(frc::Rotation2d(units::degree_t(GetHeading())),
                    m_frontLeft.GetState(), m_backLeft.GetState(),
                    m_frontRight.GetState(), m_backRight.GetState());
  frc::Pose2d frcPose(m_odometry.GetPose().Y(), -m_odometry.GetPose().X(),m_odometry.GetPose().Rotation());
  m_pField.SetRobotPose(frcPose);
  frc::SmartDashboard::PutData(&m_pField);
  if(cycle == 0){
    
    frc::SmartDashboard::PutNumber("fr state angle", m_frontRight.GetState().angle.Degrees().to<double>());
    frc::SmartDashboard::PutNumber("fr state speed", m_frontRight.GetState().speed.to<double>());
    frc::SmartDashboard::PutNumber("fr encoder ticks", fabs(m_frontRight.getDriveEncoder()));
  }else if(cycle == 1){
    frc::SmartDashboard::PutNumber("fl state angle", m_frontLeft.GetState().angle.Degrees().to<double>());
    frc::SmartDashboard::PutNumber("fl state speed", m_frontLeft.GetState().speed.to<double>());
    frc::SmartDashboard::PutNumber("fl encoder ticks", fabs(m_frontLeft.getDriveEncoder()));
  }else if(cycle == 2){
    frc::SmartDashboard::PutNumber("br state angle", m_backRight.GetState().angle.Degrees().to<double>());
    frc::SmartDashboard::PutNumber("br state speed", m_backRight.GetState().speed.to<double>());
    frc::SmartDashboard::PutNumber("br encoder ticks", fabs(m_backRight.getDriveEncoder()));
  }else if(cycle == 3){
    frc::SmartDashboard::PutNumber("bl state angle", m_backLeft.GetState().angle.Degrees().to<double>());
    frc::SmartDashboard::PutNumber("bl state speed", m_backLeft.GetState().speed.to<double>());
    frc::SmartDashboard::PutNumber("bl encoder ticks", fabs(m_backLeft.getDriveEncoder()));
  }else{
    frc::SmartDashboard::PutNumber("robot speed", sqrt((GetRobotVelocity().vx *GetRobotVelocity().vx +GetRobotVelocity().vy*GetRobotVelocity().vy).to<double>())*metersToInches);
    frc::SmartDashboard::PutNumber("Odometry X", GetPose().Translation().X().to<double>()*metersToInches);
    frc::SmartDashboard::PutNumber("Odometry Y", -1.0*GetPose().Translation().Y().to<double>()*metersToInches);
    frc::SmartDashboard::PutNumber("Odometry Yaw", GetPose().Rotation().Degrees().to<double>());
    cycle = 0;
  }
  cycle++;
  
      
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative, bool percentMode) {

  // adjust to max speeds
  double base = .1;
  if(fabs(rot.to<double>()) > base){
    rot = rot * 2.2;
  }                        

  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot,
                          frc::Rotation2d(GetPose().Rotation().Degrees()))
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});
  
  kDriveKinematics.NormalizeWheelSpeeds(&states, units::meters_per_second_t(RobotParameters::k_maxSpeed));
  auto [fl, fr, bl, br] = states;
  m_frontLeft.SetDesiredState(fl, percentMode);
  m_frontRight.SetDesiredState(fr, percentMode);
  m_backLeft.SetDesiredState(bl, percentMode);
  m_backRight.SetDesiredState(br, percentMode);
}

void DriveSubsystem::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates, bool percentMode) {
  kDriveKinematics.NormalizeWheelSpeeds(&desiredStates, units::meters_per_second_t(RobotParameters::k_maxSpeed));
  m_frontLeft.SetDesiredState(desiredStates[0], percentMode);
  m_backLeft.SetDesiredState(desiredStates[1],percentMode);
  m_frontRight.SetDesiredState(desiredStates[2], percentMode);
  m_backRight.SetDesiredState(desiredStates[3], percentMode);
  
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_backLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_backRight.ResetEncoders();
}

double DriveSubsystem::GetHeading() {
  return normalizeToRange::NormalizeToRange(m_pChassisIMU.GetYaw() , -180, 180, true) * (kGyroReversed ? -1: 1);
}
void DriveSubsystem::ZeroHeading() { 
  m_pChassisIMU.Reset(); 
  }

double DriveSubsystem::GetTurnRate() {
  return m_pChassisIMU.GetRate() * (kGyroReversed ? -1. : 1.);
}

frc::Pose2d DriveSubsystem::GetPose() { return m_odometry.GetPose(); }

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(pose,
                           frc::Rotation2d(units::degree_t(GetHeading())));
}

void DriveSubsystem::DriveArc(double arcLength){
  auto [fl, fr, bl, br] = kDriveKinematics.ToSwerveModuleStates(frc::ChassisSpeeds{0_mps, 0_mps, 10000_rpm});
  m_frontRight.DriveArc(arcLength, fr.angle.Degrees().to<double>());
  m_frontLeft.DriveArc(arcLength, fl.angle.Degrees().to<double>());
  m_backRight.DriveArc(arcLength, br.angle.Degrees().to<double>());
  m_backLeft.DriveArc(arcLength, bl.angle.Degrees().to<double>());
}
void DriveSubsystem::toggleFieldCentricForJoystick(){
  m_fieldCentricForJoystick = !m_fieldCentricForJoystick;
  frc::SmartDashboard::PutBoolean("fieldCentric", m_fieldCentricForJoystick);
}
bool DriveSubsystem::getFiedCentricForJoystick(){
  return m_fieldCentricForJoystick;
}
void DriveSubsystem::tuneDrivePID(double p, double i, double d, double f){
  m_frontLeft.updateDrivePID(p, i, d, f);
  m_frontRight.updateDrivePID(p, i, d, f);
  m_backLeft.updateDrivePID(p, i, d, f);
  m_backRight.updateDrivePID(p, i, d, f);
}

void DriveSubsystem::tuneSteerPID(double p, double i, double d){
  m_frontLeft.updateSteerPID(p, i, d);
  m_frontRight.updateSteerPID(p, i, d);
  m_backLeft.updateSteerPID(p, i, d);
  m_backRight.updateSteerPID(p, i, d);
}

void DriveSubsystem::stop(){
  Drive(0_mps, 0_mps, 0_rad_per_s, false);
}


frc::SwerveModuleState DriveSubsystem::getFrontRightMotor(){
  return  m_frontRight.GetState();
}
frc::SwerveModuleState DriveSubsystem::getFrontLeftMotor(){
  return m_frontLeft.GetState();
}
frc::SwerveModuleState DriveSubsystem::getBackRightMotor(){
  return m_backRight.GetState();
}
frc::SwerveModuleState DriveSubsystem::getBackLeftMotor(){
  return m_backLeft.GetState();
}

frc::ChassisSpeeds DriveSubsystem::GetRobotVelocity(){
  return kDriveKinematics.ToChassisSpeeds(m_frontLeft.GetState(), m_backLeft.GetState(),
                    m_frontRight.GetState(), m_backRight.GetState());
}
void DriveSubsystem::setCoast(){
  m_frontRight.setCoast();
  m_frontLeft.setCoast();
  m_backRight.setCoast();
  m_backLeft.setCoast();
}
void DriveSubsystem::setBrake(){
  m_frontRight.setBrake();
  m_frontLeft.setBrake();
  m_backRight.setBrake();
  m_backLeft.setBrake();
}

void DriveSubsystem::resetDriveEncoders(){
  m_frontRight.resetDriveEncoder();
  m_frontLeft.resetDriveEncoder();
  m_backRight.resetDriveEncoder();
  m_backLeft.resetDriveEncoder();
}
