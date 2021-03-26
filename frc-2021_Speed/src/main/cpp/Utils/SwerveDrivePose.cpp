#include "Utils/SwerveDrivePose.h"
#include "Utils/MathConstants.h"
#include <frc/geometry/Translation2d.h>
SwerveDrivePose::SwerveDrivePose(
    const frc::Pose2d &pose,
    double wheelTrack,
	double wheelBase,
    double cornerStiffCoeff)

    : m_pose(pose),
    m_poseDot(0, 0, 0),
    m_kinematics(
		frc::Translation2d(units::meter_t(wheelTrack / 2.0), units::meter_t(wheelBase / 2.0)),
		frc::Translation2d(units::meter_t(wheelTrack / 2.0), units::meter_t(-wheelBase / 2.0)),
		frc::Translation2d(units::meter_t(-wheelTrack / 2.0), units::meter_t(-wheelBase / 2.0)),
		frc::Translation2d(units::meter_t(-wheelTrack / 2.0), units::meter_t(wheelBase / 2.0))),
    m_cornerStiffCoeff(cornerStiffCoeff) {
}

SwerveDrivePose::~SwerveDrivePose() {
}

void SwerveDrivePose::reset(const frc::Pose2d &pose, const PoseDot2D &poseDot) {
    m_pose = pose;
    m_poseDot = poseDot;
}

frc::Pose2d SwerveDrivePose::getPose() {
    return m_pose;
}

PoseDot2D SwerveDrivePose::getPoseDot() {
    return m_poseDot;
}

void SwerveDrivePose::update(
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
	double yawRateGyro) {

    // wheel odometry measurement
    frc::Translation2d robotDeltaDistWheelMeas;
    double robotDeltaYawWheelMeas;
    m_kinematics.forwardKinematics(
		frc::Translation2d(units::meter_t(0), units::meter_t(deltaDistFRWheel)).RotateBy(frWheelYaw),
		frc::Translation2d(units::meter_t(0), units::meter_t(deltaDistBRWheel)).RotateBy(brWheelYaw),
		frc::Translation2d(units::meter_t(0), units::meter_t(deltaDistBLWheel)).RotateBy(blWheelYaw),
		frc::Translation2d(units::meter_t(0), units::meter_t(deltaDistFLWheel)).RotateBy(flWheelYaw),
		robotDeltaDistWheelMeas,
		robotDeltaYawWheelMeas);

    // gyro measurement
    double robotDeltaYawGyroMeas = deltaYawGyro;

    // estimate lateral wheel slip
    frc::Translation2d deltaCentripAccel = (robotDeltaDistWheelMeas*(
        -deltaYawGyro * MATH_CONSTANTS_PI / 180.0)).RotateBy(
        frc::Rotation2d(units::angle::degree_t(90.0))); 
    frc::Translation2d deltaLatWheelSlip = deltaCentripAccel*(
        -robotDeltaDistWheelMeas.Norm().to<double>() / 386.1 * m_cornerStiffCoeff); // 386.1 in/s^2 = 9.81 m/s^2

    // update pose with measurements
    frc::Translation2d deltaPosRobotFrame = robotDeltaDistWheelMeas + deltaLatWheelSlip;
    frc::Rotation2d newRobotYaw = m_pose.Rotation().RotateBy(frc::Rotation2d(units::degree_t(robotDeltaYawGyroMeas)));
    frc::Translation2d deltaPosGlobalFrame = (deltaPosRobotFrame.RotateBy(m_pose.Rotation().Radians())
        + deltaPosRobotFrame.RotateBy(newRobotYaw))*0.5; // trapezoidal integration
    frc::Translation2d newRobotPos = m_pose.Translation()+deltaPosGlobalFrame;
    m_pose = frc::Pose2d(newRobotPos, newRobotYaw);

    // update pose dot with measurements
    frc::Translation2d robotVelWheelMeas;
    double robotYawRateWheelMeas;
    m_kinematics.forwardKinematics(
		frc::Translation2d(units::meter_t(0), units::meter_t(velFRWheel)).RotateBy(frWheelYaw),
		frc::Translation2d(units::meter_t(0), units::meter_t(velBRWheel)).RotateBy(brWheelYaw),
		frc::Translation2d(units::meter_t(0), units::meter_t(velBLWheel)).RotateBy(blWheelYaw),
		frc::Translation2d(units::meter_t(0), units::meter_t(velFLWheel)).RotateBy(flWheelYaw),
		robotVelWheelMeas,
		robotYawRateWheelMeas);
    m_poseDot = PoseDot2D(robotVelWheelMeas.X().to<double>(), robotVelWheelMeas.X().to<double>(), yawRateGyro);

}
