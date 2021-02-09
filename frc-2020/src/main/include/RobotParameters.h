#include "Utils/MathConstants.h"
#include <math.h>
#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H

namespace RobotParameters {
    // robot main
	static constexpr unsigned k_updateRate = 50; // Hz

    // drivetrain .0625+ .11 = .41
    static constexpr double k_wheelBase = 22.938341*.0254; // in
    static constexpr double k_wheelTrack = 27.438*.0254; // in
    static constexpr double k_wheelLeverArm = sqrt(std::pow(k_wheelBase/2,2) + std::pow(k_wheelTrack/2,2));
    static constexpr double k_wheelRad = (3.79/2)*.0254*1.03; // in
    static constexpr double k_maxSpeed = 3.75; //1; 
    static constexpr double k_maxAccel = 250*.0254; //1; // m/s^2// 200*.0254
    static constexpr double k_maxDeccel = -75*.0254;//was 50in
    static constexpr double k_steerEncoderToWheelGearRatio = 1; // gear ratio from steer encoder to wheel
    static constexpr double k_driveMotorGearRatio = 30.0/11.0 *3.0;
    static constexpr double k_ticksPerRev= 2048.0;//ticks per 100ms
    static constexpr double k_driveMotorEncoderTicksToMPS = (1/k_ticksPerRev)*(1/k_driveMotorGearRatio)*k_wheelRad*3.14159265*2*10;
    static constexpr double k_driveMotorEncoderTicksToMeters = (1/k_ticksPerRev)*(1/k_driveMotorGearRatio)*k_wheelRad*3.14159265*2;
    static constexpr double k_minRobotVelocity = .07;//3*.0254;
    static constexpr double k_minRobotYawRate = 10;//3*.0254;
    static constexpr double k_driveWheelSlotError = 0.002;
    static constexpr double k_robotWidth = 38.5;
    static constexpr double k_robotLength = 34.5;
    static constexpr double k_maxYawRate = k_maxSpeed / k_wheelLeverArm *180/MATH_CONSTANTS_PI;
    static constexpr double k_maxYawAccel = k_maxAccel / k_wheelLeverArm*180/MATH_CONSTANTS_PI;
    static constexpr double k_maxYawDeccel = k_maxDeccel / k_wheelLeverArm*180/MATH_CONSTANTS_PI;
    static constexpr double k_minYawRate = k_minRobotVelocity / k_wheelLeverArm *180 / MATH_CONSTANTS_PI;
    // static constexpr double k_driveMotorEncoderMPSToRPM  = (RobotParameters::k_driveMotorGearRatio/(RobotParameters::k_wheelRad*3.14159265*2))*60;


    //pathfollowing 
    static constexpr double  k_maxCentripAccel = 10.0;//10

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

    //shooter
    static constexpr double k_shooterP = 1.09;
    static constexpr double k_shooterI = 0.001;
    static constexpr double k_shooterD = 0.05;
    static constexpr double k_shooterF = .0481;
    
    static constexpr double k_climbP = 0.8;
    static constexpr double k_climbI = 0.0;
    static constexpr double k_climbD = 4.8;
    static constexpr double k_climbF = 0;

    static constexpr double k_feederP = 0.8;
    static constexpr double k_feederI = 0;
    static constexpr double k_feederD = 4.8;
    static constexpr double k_feederF = 0;

    static constexpr double k_limeLightP = 2.1;//4.9
    static constexpr double k_limeLightI = 1;
    static constexpr double k_limeLightD = 0;
    static constexpr double k_limeLightIZone = 5;

    static constexpr double k_limeLightDriveP = 1.4;//4.9
    static constexpr double k_limeLightDriveI = .6;//.1
    static constexpr double k_limeLightDriveD = 0.1;

}

#endif // ROBOT_PARAMETERS_H