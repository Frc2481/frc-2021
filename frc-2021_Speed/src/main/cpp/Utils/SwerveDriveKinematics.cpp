#include "Utils/SwerveDriveKinematics.h"
#include "Utils/MathConstants.h"
#include <frc/geometry/Translation2d.h>

SwerveDriveKinematics::SwerveDriveKinematics(
	frc::Translation2d frLeverArm,
	frc::Translation2d brLeverArm,
	frc::Translation2d blLeverArm,
	frc::Translation2d flLeverArm)

    : m_frLeverArm(frLeverArm),
	  m_brLeverArm(brLeverArm),
	  m_blLeverArm(blLeverArm),
	  m_flLeverArm(flLeverArm) {
}

SwerveDriveKinematics::~SwerveDriveKinematics() {
}

void SwerveDriveKinematics::forwardKinematics(
	frc::Translation2d frWheelVel,
	frc::Translation2d brWheelVel,
	frc::Translation2d blWheelVel,
	frc::Translation2d flWheelVel,
	frc::Translation2d &robotVel,
	double &robotYawRate) {

	double robotYawRate1 = -(frWheelVel.X().to<double>() - blWheelVel.X().to<double>()) / (m_frLeverArm.Y().to<double>() - m_blLeverArm.Y().to<double>());
	double robotYawRate2 = (frWheelVel.Y().to<double>() - blWheelVel.Y().to<double>()) / (m_frLeverArm.X().to<double>() - m_blLeverArm.X().to<double>());
	double robotYawRate3 = -(flWheelVel.X().to<double>() - brWheelVel.X().to<double>()) / (m_flLeverArm.Y().to<double>() - m_brLeverArm.Y().to<double>());
	double robotYawRate4 = (flWheelVel.Y().to<double>() - brWheelVel.Y().to<double>()) / (m_flLeverArm.X().to<double>() - m_brLeverArm.X().to<double>());
	robotYawRate = (robotYawRate1 + robotYawRate2 + robotYawRate3 + robotYawRate4) / 4.0;

	frc::Translation2d robotVel1 = frWheelVel - frc::Translation2d(-robotYawRate * m_frLeverArm.Y(), robotYawRate * m_frLeverArm.X());
	frc::Translation2d robotVel2 = brWheelVel - frc::Translation2d(-robotYawRate * m_brLeverArm.Y(), robotYawRate * m_brLeverArm.X());
	frc::Translation2d robotVel3 = blWheelVel - frc::Translation2d(-robotYawRate * m_blLeverArm.Y(), robotYawRate * m_blLeverArm.X());
	frc::Translation2d robotVel4 = flWheelVel - frc::Translation2d(-robotYawRate * m_flLeverArm.Y(), robotYawRate * m_flLeverArm.X());
	robotVel = (robotVel1 + robotVel2 + robotVel3 + robotVel4)*(1.0 / 4.0);

	robotYawRate *= 180.0 / MATH_CONSTANTS_PI; // convert to deg/s
}

void SwerveDriveKinematics::inverseKinematics(
	frc::Translation2d robotVel,
	double robotYawRate,
	frc::Translation2d &frWheelVel,
	frc::Translation2d &brWheelVel,
	frc::Translation2d &blWheelVel,
	frc::Translation2d &flWheelVel) {

	robotYawRate *= MATH_CONSTANTS_PI / 180.0; // convert to rad/s
	frWheelVel = frc::Translation2d(-robotYawRate * m_frLeverArm.Y(), robotYawRate * m_frLeverArm.X()) + robotVel;
	brWheelVel = frc::Translation2d(-robotYawRate * m_brLeverArm.Y(), robotYawRate * m_brLeverArm.X()) + robotVel;
	blWheelVel = frc::Translation2d(-robotYawRate * m_blLeverArm.Y(), robotYawRate * m_blLeverArm.X()) + robotVel;
	flWheelVel = frc::Translation2d(-robotYawRate * m_flLeverArm.Y(), robotYawRate * m_flLeverArm.X()) + robotVel;
}
