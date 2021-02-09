#ifndef SWERVE_DRIVE_POSE_H
#define SWERVE_DRIVE_POSE_H

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include "Utils/PoseDot2D.h"
#include "Utils/SwerveDriveKinematics.h"

// +x = field right
// +y = field forward
// +yaw = CCW, zero is field forward

class SwerveDrivePose {
public:
	SwerveDrivePose(
        const frc::Pose2d &pose,
        double wheelTrack,
		double wheelBase,
        double cornerStiffCoeff);
    ~SwerveDrivePose();

    void reset(const frc::Pose2d &pose, const PoseDot2D &poseDot);
    frc::Pose2d getPose();
    PoseDot2D getPoseDot();

    //////////////////////////////////////////////////////////////////////
    // @brief update pose estimate
    // @param deltaDistFRWheel - change in front right wheel linear distance (in)
    // @param deltaDistBRWheel - change in back right wheel linear distance (in)
    // @param deltaDistBLWheel - change in back left wheel linear distance (in)
    // @param deltaDistFLWheel - change in front left wheel linear distance (in)
    // @param deltaYawGyro - change in gyro yaw (deg)
    // @param velFRWheel - front right wheel linear velocity (in/s)
    // @param velBRWheel - back right wheel linear velocity (in/s)
    // @param velBLWheel - back left wheel linear velocity (in/s)
    // @param velFLWheel - front left wheel linear velocity (in/s)
    // @param yawRateGyro - yaw rate gyro (deg/s)
    //////////////////////////////////////////////////////////////////////
    void update(
        double deltaDistFRWheel,
		double deltaDistBRWheel,
		double deltaDistBLWheel,
		double deltaDistFLWheel,
		frc::Rotation2d frWheelYaw,
		frc::Rotation2d brWheelYaw,
		frc::Rotation2d blWheelYaw,
		frc::Rotation2d flWheelYaw,
        double deltaYawGyro,
        double velFRWheel,
        double velBRWheel,
		double velBLWheel,
		double velFLWheel,
        double yawRateGyro);

private:
    frc::Pose2d m_pose;
    PoseDot2D m_poseDot;
    SwerveDriveKinematics m_kinematics;
    double m_cornerStiffCoeff;
};

#endif // SWERVE_DRIVE_POSE_H
