#ifndef SWERVE_DRIVE_KINEMATICS_H
#define SWERVE_DRIVE_KINEMATICS_H

#include <frc/geometry/Translation2d.h>
// +x = robot right
// +y = robot forward
// +yaw = CCW, zero is robot forward

class SwerveDriveKinematics {
public:
	SwerveDriveKinematics(
		frc::Translation2d frLeverArm,
		frc::Translation2d brLeverArm,
		frc::Translation2d blLeverArm,
		frc::Translation2d flLeverArm);
    ~SwerveDriveKinematics();

    //////////////////////////////////////////////////////////////////////
    // @brief calculate robot velocity and yaw rate from wheel velocities
    // @param frWheelVel - front right wheel velocity (in/s)
    // @param brWheelVel - back right wheel velocity (in/s)
    // @param blWheelVel - back left wheel velocity (in/s)
    // @param flWheelVel - front left wheel velocity (in/s)
    // @param robotVel - robot velocity (in/s)
    // @param robotYawRate - robot yaw rate (deg/s)
    //////////////////////////////////////////////////////////////////////
    void forwardKinematics(
    	frc::Translation2d frWheelVel,
		frc::Translation2d brWheelVel,
		frc::Translation2d blWheelVel,
		frc::Translation2d flWheelVel,
		frc::Translation2d &robotVel,
		double &robotYawRate);

    //////////////////////////////////////////////////////////////////////
    // @brief calculate wheel velocities from robot velocity and yaw rate
    // @param robotVel - robot velocity (in/s)
    // @param robotYawRate - robot yaw rate (deg/s)
    // @param frWheelVel - front right wheel velocity (in/s)
	// @param brWheelVel - back right wheel velocity (in/s)
	// @param blWheelVel - back left wheel velocity (in/s)
	// @param flWheelVel - front left wheel velocity (in/s)
    //////////////////////////////////////////////////////////////////////
    void inverseKinematics(
		frc::Translation2d robotVel,
		double robotYawRate,
		frc::Translation2d &frWheelVel,
		frc::Translation2d &brWheelVel,
		frc::Translation2d &blWheelVel,
		frc::Translation2d &flWheelVel);

private:
    frc::Translation2d m_frLeverArm;
    frc::Translation2d m_brLeverArm;
    frc::Translation2d m_blLeverArm;
    frc::Translation2d m_flLeverArm;
};

#endif // SWERVE_DRIVE_KINEMATICS_H
