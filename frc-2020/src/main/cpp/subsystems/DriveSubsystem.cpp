/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <units/units.h>

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
                  "FL_STEER_MOTOR_ENCODER"},

      m_rearLeft{
                  FalconIDs::kRearLeftDriveMotorID,       
                  TalonIDs::kRearLeftTurningMotorID,
                  kRearLeftDriveEncoderReversed, 
                  kRearLeftTurningEncoderReversed,
                  "BL_STEER_MOTOR_ENCODER"},

      m_frontRight{
                  FalconIDs::kFrontRightDriveMotorID,       
                  TalonIDs::kFrontRightTurningMotorID,
                  kFrontRightDriveEncoderReversed, 
                  kFrontRightTurningEncoderReversed,
                  "FR_STEER_MOTOR_ENCODER"},

      m_rearRight{
                  FalconIDs::kRearRightDriveMotorID,       
                  TalonIDs::kRearRightTurningMotorID,
                  kRearRightDriveEncoderReversed, 
                  kRearRightTurningEncoderReversed,
                  "BR_STEER_MOTOR_ENCODER"},
        //1097Setting 

      m_odometry{kDriveKinematics,
                 frc::Rotation2d(units::degree_t(0)),
                 frc::Pose2d()},

      m_pChassisIMU{frc::SPI::kMXP}{
    //     std::remove("home/lvuser/ActualPath.csv");
    // m_File.open("home/lvuser/ActualPath.csv");
      }


void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(frc::Rotation2d(units::degree_t(GetHeading())),
                    m_frontLeft.GetState(), m_rearLeft.GetState(),
                    m_frontRight.GetState(), m_rearRight.GetState());
  // m_File << GetRobotVelocity().vx.to<double>() <<",";
  // m_File << GetRobotVelocity().vy.to<double>() <<"\n";
  frc::SmartDashboard::PutNumber("robot speed", sqrt((GetRobotVelocity().vx *GetRobotVelocity().vx +GetRobotVelocity().vy*GetRobotVelocity().vy).to<double>()));
  frc::SmartDashboard::PutNumber("fr state angle", m_frontRight.GetState().angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("fr state speed", m_frontRight.GetState().speed.to<double>());
  frc::SmartDashboard::PutNumber("fl state angle", m_frontLeft.GetState().angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("fl state speed", m_frontLeft.GetState().speed.to<double>());
  frc::SmartDashboard::PutNumber("br state angle", m_rearRight.GetState().angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("br state speed", m_rearRight.GetState().speed.to<double>());
  frc::SmartDashboard::PutNumber("bl state angle", m_rearLeft.GetState().angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("bl state speed", m_rearLeft.GetState().speed.to<double>());
  frc::SmartDashboard::PutNumber("Odometry X", GetPose().Translation().X().to<double>()*39.3701);//
  frc::SmartDashboard::PutNumber("Odometry Y", GetPose().Translation().Y().to<double>()*39.3701);//*39.3701
  frc::SmartDashboard::PutNumber("Odometry Yaw", GetPose().Rotation().Degrees().to<double>());    
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative, bool percentMode, const std::string &name) {
  // printf("%s is using the Drive command\n",name.c_str());
  frc::SmartDashboard::PutNumber("passed in value",ySpeed.to<double>());                        
  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot,
                          frc::Rotation2d(GetPose().Rotation().Degrees()))
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});
frc::SmartDashboard::PutNumber("driveMotorVelpreNormSpeed",states[0].speed.to<double>());
frc::SmartDashboard::PutNumber("driveMotorVelpreNormAngle",states[0].angle.Degrees().to<double>());
  kDriveKinematics.NormalizeWheelSpeeds(&states, units::meters_per_second_t(RobotParameters::k_maxSpeed));
  frc::SmartDashboard::PutNumber("driveMotorVelNorm",states[0].speed.to<double>());
  auto [fl, fr, bl, br] = states;
  m_frontLeft.SetDesiredState(fl, percentMode);
  m_frontRight.SetDesiredState(fr, percentMode);
  m_rearLeft.SetDesiredState(bl, percentMode);
  m_rearRight.SetDesiredState(br, percentMode);
  frc::SmartDashboard::PutNumber("fr", fr.speed.to<double>());
  frc::SmartDashboard::PutNumber("fl", fl.speed.to<double>());
  frc::SmartDashboard::PutNumber("br", br.speed.to<double>());
  frc::SmartDashboard::PutNumber("bl", bl.speed.to<double>());
  // frc::SmartDashboard::PutNumber("RobotVelocityX",GetRobotVelocity().vy.to<double>());
  // frc::SmartDashboard::PutNumber("RobotVelocityY",GetRobotVelocity().vx.to<double>());
  // printf("fr angle: %3f, fl angle: %3f, br angle: %3f, bl angle: %3f\n", 
          // m_frontRight.GetState().angle.Degrees().to<double>(),
          // m_frontLeft.GetState().angle.Degrees().to<double>(),
          // m_rearRight.GetState().angle.Degrees().to<double>(),
          // m_rearLeft.GetState().angle.Degrees().to<double>());

  
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates, bool percentMode) {
  kDriveKinematics.NormalizeWheelSpeeds(&desiredStates,
                                        AutoConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0], percentMode);
  m_rearLeft.SetDesiredState(desiredStates[1],percentMode);
  m_frontRight.SetDesiredState(desiredStates[2], percentMode);
  m_rearRight.SetDesiredState(desiredStates[3], percentMode);
  
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
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
  
  // frc::SmartDashboard::PutNumber("Odometry X", GetPose().Translation().X().to<double>());
  // frc::SmartDashboard::PutNumber("Odometry Y", GetPose().Translation().Y().to<double>());
  // frc::SmartDashboard::PutNumber("Odometry Yaw", GetPose().Rotation().Degrees().to<double>());

}

void DriveSubsystem::DriveArc(double arcLength){
  auto [fl, fr, bl, br] = kDriveKinematics.ToSwerveModuleStates(frc::ChassisSpeeds{0_mps, 0_mps, 10000_rpm});
  m_frontRight.DriveArc(arcLength, fr.angle.Degrees().to<double>());
  m_frontLeft.DriveArc(arcLength, fl.angle.Degrees().to<double>());
  m_rearRight.DriveArc(arcLength, br.angle.Degrees().to<double>());
  m_rearLeft.DriveArc(arcLength, bl.angle.Degrees().to<double>());
}
void DriveSubsystem::toggleFieldCentricForJoystick(){
  if(m_fieldCentricForJoystick){
    m_fieldCentricForJoystick =  false;
  }else{
    m_fieldCentricForJoystick = true;
  }
  frc::SmartDashboard::PutBoolean("fieldCentric", m_fieldCentricForJoystick);
}
bool DriveSubsystem::getFiedCentricForJoystick(){
  return m_fieldCentricForJoystick;
}
void DriveSubsystem::tuneDrivePID(double p, double i, double d, double f){
  m_frontLeft.updateDrivePID(p, i, d, f);
  m_frontRight.updateDrivePID(p, i, d, f);
  m_rearLeft.updateDrivePID(p, i, d, f);
  m_rearRight.updateDrivePID(p, i, d, f);
}

void DriveSubsystem::tuneSteerPID(double p, double i, double d){
  m_frontLeft.updateSteerPID(p, i, d);
  m_frontRight.updateSteerPID(p, i, d);
  m_rearLeft.updateSteerPID(p, i, d);
  m_rearRight.updateSteerPID(p, i, d);
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
  return m_rearRight.GetState();
}
frc::SwerveModuleState DriveSubsystem::getBackLeftMotor(){
  return m_rearLeft.GetState();
}

frc::ChassisSpeeds DriveSubsystem::GetRobotVelocity(){
  return kDriveKinematics.ToChassisSpeeds(m_frontLeft.GetState(), m_rearLeft.GetState(),
                    m_frontRight.GetState(), m_rearRight.GetState());
}
void DriveSubsystem::setCoast(){
  m_frontRight.setCoast();
  m_frontLeft.setCoast();
  m_rearRight.setCoast();
  m_rearLeft.setCoast();
}
void DriveSubsystem::setBrake(){
  m_frontRight.setBrake();
  m_frontLeft.setBrake();
  m_rearRight.setBrake();
  m_rearLeft.setBrake();
}