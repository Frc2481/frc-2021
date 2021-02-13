/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/InstantCommand.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveSubsystem.h"
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class LimelightUpdatePose
    : public frc2::CommandHelper<frc2::InstantCommand, LimelightUpdatePose> {
 private:
  DriveSubsystem* m_driveTrain;
 public:
  LimelightUpdatePose(DriveSubsystem* driveTrain){
    m_driveTrain = driveTrain;
    AddRequirements(m_driveTrain);
  }

  void Initialize() override{
    double tx = NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0);
    double ty = (nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty",0)-LimeLightConstants::kLimeLightAngle + LimeLightConstants::kLimeLightHardWarePanAngle)*MATH_CONSTANTS_PI/180;
    double height = LimeLightConstants::kTargetHeight - LimeLightConstants::kLimeLightHeight;
    // frc::Translation2d targetPose = Translation2d(0, 0); //target on one side of field//TODO check coordinate frame
    double headingToTarget = m_driveTrain->GetPose().Rotation().Radians().to<double>()+ tx*MATH_CONSTANTS_PI/2 + MATH_CONSTANTS_PI/2;
    double distance = height/tan(ty);
    double distanceX = distance*sin(headingToTarget);
    double distanceY = distance*cos(headingToTarget);
    frc::SmartDashboard::PutNumber("Lime light X", distanceX);
    frc::SmartDashboard::PutNumber("Lime light Y", distanceY);
    frc::SmartDashboard::PutNumber("Lime light yaw", headingToTarget);
    frc::Translation2d newPose = frc::Translation2d(units::meter_t(15.98), units::meter_t(8.21)) - frc::Translation2d(units::meter_t(distanceX), units::meter_t
    (distanceY));

    m_driveTrain->ResetOdometry(frc::Pose2d(newPose, m_driveTrain->GetPose().Rotation()));
  }

};
