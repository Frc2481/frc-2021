/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <ctre/Phoenix.h>
#include <frc/Encoder.h>
// #include <frc/Spark.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <wpi/math>

#include "Constants.h"
#include "components/CTREMagEncoder.h"
#include "components/MotorPositionController.h"
#include "RobotParameters.h"
#include "components/CommonMotorController.h"
#include "components/TalonSRXMotorController.h"
#include "components/TalonFXMotorController.h"

class SwerveModule {
  using radians_per_second_squared_t =
      units::compound_unit<units::radians,
                           units::inverse<units::squared<units::second>>>;

 public:
  SwerveModule(int driveMotorID, int turningMotorID,
               bool driveEncoderReversed, bool turningEncoderReversed, bool turningMotorReversed,const std::string &name);

  frc::SwerveModuleState GetState();

  void SetDesiredState(frc::SwerveModuleState& state, bool percentMode);

  void ResetEncoders();
    void updateSteerPID(double p, double i, double d);
    void updateDrivePID(double p, double i, double d, double f);
    void setCoast();
    void setBrake();
    void DriveArc(double arcLength, double wheelAngle);
    double getDriveEncoder();
    void resetDriveEncoder();
 private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.

  static constexpr units::radians_per_second_t kModuleMaxAngularVelocity =
      units::radians_per_second_t(3.0 * 2.0*wpi::math::pi);  // radians per second
  static constexpr units::unit_t<radians_per_second_squared_t>
      kModuleMaxAngularAcceleration =
          units::unit_t<radians_per_second_squared_t>(
              wpi::math::pi * 2.0 * 12.0);  // radians per second squared

  TalonFXMotorController* m_driveMotor;
  TalonSRXMotorController* m_turningMotor;
  CTREMagEncoder* m_turningEncoder;
  MotorPositionController* m_turningMotorController;

  bool m_reverseDriveEncoder;
  bool m_reverseTurningEncoder;
  bool m_turningMotorReversed;
  std::string m_name;

  frc2::PIDController m_drivePIDController{
      0.0, 0.0, 0.0};
  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      1.0,
      0.0,
      0.05,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};
      
    
};
