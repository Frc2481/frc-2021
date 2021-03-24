#include "Utils/MathConstants.h"
#include <math.h>
#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H

namespace RobotParameters {
    // robot main
	static constexpr unsigned k_updateRate = 50; // Hz

    // drivetrain .0625+ .11 = .41
    static constexpr double k_wheelBase = 13.5*.0254; // m
    static constexpr double k_wheelTrack = 13.5*.0254; // m
    static constexpr double k_robotWidth = 24; //in
    static constexpr double k_robotLength = 24; //in
    static constexpr double k_wheelLeverArm = sqrt(std::pow(k_wheelBase/2,2) + std::pow(k_wheelTrack/2,2));
    static constexpr double k_wheelRad = (4.25)*.0254 / 2.0* .94444; // m *constant
    static constexpr double k_maxSpeed = 240*.0254; //24*12
    static constexpr double k_feedForwardMaxSpeed = 260*.0254;
    static constexpr double k_maxAccel = 450*.0254; //250//177.64
    static constexpr double k_maxDeccel = -456*.0254;//456was 150in//200//-177.64
    static constexpr double k_steerEncoderToWheelGearRatio = 1; // gear ratio from steer encoder to wheel
    static constexpr double k_driveMotorGearRatio = (26.0 / 16.0) * 3.0;
    static constexpr double k_ticksPerRev = 2048.0;//ticks per 100ms
    static constexpr double k_driveMotorEncoderTicksToMPS = (1/k_ticksPerRev)*(1/k_driveMotorGearRatio)*k_wheelRad*3.14159265*2*10;//
    static constexpr double k_driveMotorEncoderTicksToMeters = (1/k_ticksPerRev)*(1/k_driveMotorGearRatio)*k_wheelRad*3.14159265*2;
    static constexpr double k_minRobotVelocity = .07;//3*.0254;
    static constexpr double k_minRobotYawRate = 10;//3*.0254;
    static constexpr double k_driveWheelSlotError = 0.002;
    static constexpr double k_maxYawRate = k_maxSpeed / k_wheelLeverArm *180/MATH_CONSTANTS_PI;
    static constexpr double k_maxYawAccel = k_maxAccel / k_wheelLeverArm*180/MATH_CONSTANTS_PI;
    static constexpr double k_maxYawDeccel = k_maxDeccel / k_wheelLeverArm*180/MATH_CONSTANTS_PI;
    static constexpr double k_minYawRate = k_minRobotVelocity / k_wheelLeverArm *180 / MATH_CONSTANTS_PI;

    //pathfollowing 
    static constexpr double  k_maxCentripAccel = 15.0;//20//10

    // // steer motors
    static constexpr double k_steerMotorControllerKp = 3;
    static constexpr double k_steerMotorControllerKi = 0;
    static constexpr double k_steerMotorControllerKd = 40;
    static constexpr double k_steerMotorControllerKsf = 0;
    static constexpr double k_steerMotorControllerKv = 0;
    static constexpr double k_steerMotorControllerKap = 0;
    static constexpr double k_steerMotorControllerKan = 0;


    // encoders
    static constexpr unsigned k_ctreMagEncoderTicksPerRev = 4096;
    static constexpr unsigned k_grayhillEncoderTicksPerRev = 512;
    static constexpr unsigned k_falconFXEncoderTicksPerRev = 2048;
}

#endif // ROBOT_PARAMETERS_H