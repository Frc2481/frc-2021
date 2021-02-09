// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// #pragma once

// #include <frc2/command/CommandBase.h>
// #include <frc2/command/CommandHelper.h>
// #include <vector>
// #include <limits>
// #include <algorithm>
// #include "RobotParameters.h"
// #include "Utils/SwerveDrivePathGenerator.h"
// #include "Utils/Sign.h"
// #include "Utils/PIDVAController.h"
// #include "subsystems/DriveSubsystem.h"
// #include <frc/geometry/Translation2d.h>
// #include <frc/geometry/Rotation2d.h>
// #include "Utils/PoseDot2D.h"
// #include <frc/smartdashboard/SmartDashboard.h>
// #include <cmath>
// #include <iostream>
// #include <fstream>
// #include "Utils/MathConstants.h"
// #include <frc/controller/PIDController.h>
// #include "networktables/NetworkTableInstance.h"
// /**
//  * An example command.
//  *
//  * <p>Note that this extends CommandHelper, rather extending CommandBase
//  * directly; this is crucially important, or else the decorator functions in
//  * Command will *not* work!
//  */

// //0.3048 feet to meters
// class ExtreamePathFollower
//     : public frc2::CommandHelper<frc2::CommandBase, ExtreamePathFollower> {
//  public:
//   ExtreamePathFollower(std::vector<SwerveDrivePathGenerator::waypoint_t> &waypoints,
// 		bool isReverse,
// 		double targetZone,
//     DriveSubsystem* driveTrain): m_lastPointReached(false),
// 		m_distToEnd(std::numeric_limits<double>::infinity()),
// 		m_targetZone(targetZone),
// 		m_time(0){
     
//       m_pDrivetrain = driveTrain;
	  
//      AddRequirements(m_pDrivetrain);
// 		// generate path
// 		SwerveDrivePathGenerator pathGenerator(
// 			waypoints,
// 			RobotParameters::k_updateRate,
// 			RobotParameters::k_wheelTrack,
// 			RobotParameters::k_wheelBase,
// 			RobotParameters::k_maxSpeed,
// 			RobotParameters::k_maxAccel,
// 			RobotParameters::k_maxDeccel,
// 			RobotParameters::k_maxCentripAccel
// 			);
// 		pathGenerator.setIsReverse(isReverse);
// 		pathGenerator.generatePath();
// 		pathGenerator.setPathFilename("/home/lvuser/SwerveDrive.csv");
// 		pathGenerator.writeTempPathToCSV();
// 		pathGenerator.writePathToCSV();
// 		pathGenerator.writeComboPathToCSV();
// 		m_path = pathGenerator.getFinalPath();
// 		// printf("generated paths");
// 		m_path.erase(m_path.begin()); // workaround of vel and accel zero at start
// 		// m_path.begin();
// 		// ensure target zone is achievable
// 		if(m_targetZone < .0254)
// 		{
// 			m_targetZone = .0254;
// 		}
// 		// frc::SmartDashboard::PutNumber("path generated time",m_endTime-m_startTime);
// 	}
// 	void Initialize() {
// 		std::remove("home/lvuser/ActualPath.csv");
// 		m_File.open("home/lvuser/ActualPath.csv");
// 		m_File << "time (s), xPos (m), yPos (m), yaw (deg), velX (m/s), velY (m), YawRate (deg/s), currentTime(s), currentX(m), currentY(m), CurrentYaw(deg), skipped, actualYVel (m/s), actualXVel(m/s), diffAngle\n";
// 		frc::SmartDashboard::PutBoolean("auto interrupted", false);
// 		frc::SmartDashboard::PutBoolean("end auto", false);
// 		frc::SmartDashboard::PutBoolean("auto timed out",false);
		
// 		frc::SmartDashboard::PutNumber("path generated time",m_path.end()->time + RobotParameters::k_pathFollowerTimeoutAllowance);
// 		// frc::SmartDashboard::PutNumber("start time", m_startTime);
// 		m_timer.Start();
// 		// frc::SmartDashboard::PutNumber("time", m_timer.Get());
// 		m_endTime = m_path.end()->time + RobotParameters::k_pathFollowerTimeoutAllowance+m_timer.Get();
// 		//for testing
// 		m_timer.Stop();  

// 	}

// 	void Execute() {
// 		m_tv = (bool)NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv",0);
// 		// frc::SmartDashboard::PutNumber("time", m_timer.Get());
// 		// update pose
// 		frc::Pose2d pose = m_pDrivetrain->GetPose();

// 		// find closest point on path
// 		if(m_path.empty()) {
// 			// printf("empty");
// 			m_lastPointReached = true;
// 			m_distToEnd = 0;
// 			return;
// 		}

// 		frc::Translation2d vectClosestPointToRobot;
// 		double distToClosestPoint = std::numeric_limits<double>::infinity();
// 		int driftingAway = 0;
// 		int skipped = 0;
// 		double diffAngle = 0;
// 		// could start search at last closest point for efficiency gain but decided not to in case
// 		// pose jumps backward along path and want to start tracking path from new point on path.
// 		// could end search after a few iterations for efficiency gain but decided not to in case
// 		// pose jumps forward along path and want to start tracking path from new point on path.
// 		for(int j = m_pathIndex; j < m_path.size(); j++){
// 			frc::Translation2d vectPointToRobot = pose.Translation() - frc::Translation2d(units::meter_t(m_path[j].xPos),units::meter_t(m_path[j].yPos));
// 			double distToPoint = vectPointToRobot.Norm().to<double>();
// 			double dirToPointRad = atan2(-vectPointToRobot.Y().to<double>(),-vectPointToRobot.X().to<double>());
// 			double pointDirRad = atan2(m_path[j].yVel, m_path[j].xVel);
// 			// printf("dirToPointRad: %f, pointDirRad: %f\n", dirToPointRad, pointDirRad);
// 			frc::SmartDashboard::PutNumber("distToPoint", fabs(dirToPointRad - pointDirRad));
// 			//find closest point that is a specific time ahead so you dont grab points to far ahead in the path
// 			diffAngle = fabs(dirToPointRad - pointDirRad);
// 			if(MATH_CONSTANTS_PI < diffAngle){
// 				diffAngle = 2*MATH_CONSTANTS_PI - diffAngle;
// 			}
// 			if(distToPoint < distToClosestPoint){
// 				if(MATH_CONSTANTS_PI/2 > diffAngle){
// 					m_pathIndex = j;
// 					distToClosestPoint = distToPoint;
// 					driftingAway = 0;
// 				}else{
// 					skipped++;
// 				}
// 			}else{
// 				driftingAway++;
// 				if(driftingAway >= 5){
// 					// printf("------------------broke out of path-----------------------------");
// 					break;
// 				}
// 			}
// 		}
		
// 		// check if last point reached
// 		if(m_pathIndex == m_path.size()-1 || distToClosestPoint == std::numeric_limits<double>::infinity() ) {
// 			// printf("ended 120");
// 			m_lastPointReached  = true;
// 		}

// 		m_time += 1.0/RobotParameters::k_updateRate;

// 		double robotYawRate = m_path[m_pathIndex].yawRate;
// 		double robotXVel = m_path[m_pathIndex].xVel;
// 		double robotYVel = m_path[m_pathIndex].yVel;
// 		double poseX = m_path[m_pathIndex].xPos;
// 		double poseY = m_path[m_pathIndex].yPos;
// 		double followerYaw = m_path[m_pathIndex].yaw;
// 		// robotXVel = m_drivePIDController.Calculate(poseX - pose.Translation().X().to<double>());
// 		// robotYVel = m_drivePIDController.Calculate(poseY- pose.Translation().Y().to<double>());
		
// 		// if(fabs(robotYawRate) >= RobotParameters::k_rotateToAngleMaxYawRate){
// 		// 	robotYawRate = RobotParameters::k_rotateToAngleMaxYawRate;
// 		// }
// 		double actualYaw = pose.Rotation().Degrees().to<double>();
// 		m_File << m_path[m_pathIndex].time << ",";
// 		m_File << poseX << ",";
// 		m_File << poseY << ",";
// 		m_File << followerYaw << ",";
// 		m_File << robotXVel << ",";
// 		m_File << robotYVel << ",";
// 		m_File << robotYawRate << ",";
// 		m_File << m_time << ",";
// 		m_File << pose.Translation().X().to<double>() << ",";
// 		m_File << pose.Translation().Y().to<double>() << ",";
// 		m_File << actualYaw << ",";
// 		m_File << skipped << ",";
// 		m_File << diffAngle << ",";
// 		// m_File << m_pDrivetrain->GetRobotVelocity().vx.to<double>()*cos(actualYaw) << ",";//"field relative"
// 		// m_File << m_pDrivetrain->GetRobotVelocity().vy.to<double>()*sin(actualYaw) << ",";
// 		m_File << m_pDrivetrain->GetRobotVelocity().vx.to<double>() << ",";//"field relative"
// 		m_File << m_pDrivetrain->GetRobotVelocity().vy.to<double>() << ",";
// 		m_File << "\n";
// 		frc::SmartDashboard::PutNumber("robotYVel follower", robotYVel);
// 		frc::SmartDashboard::PutNumber("x vel follower", robotXVel);
// 		frc::SmartDashboard::PutNumber("y vel follower", robotYVel);
// 		frc::SmartDashboard::PutNumber("yaw rate follower", robotYawRate);
// 		frc::SmartDashboard::PutBoolean("end auto", (m_distToEnd < m_targetZone) || m_lastPointReached);
// 		frc::SmartDashboard::PutNumber("x follower pos", poseX);
// 		frc::SmartDashboard::PutNumber("y follower pos", poseY);
// 		frc::SmartDashboard::PutNumber("follower yaw", followerYaw);
// 		if(m_tv && m_autoTargetEngaged){
// 			double m_turnInput = NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0)*wpi::math::pi/180; //get target angle
// 			auto yawRate = m_turningPIDController.Calculate(units::radian_t((m_turnInput))); //pid tune motor controller
// 			if(fabs(m_turnInput)< m_targetZone){// set turning to zero if in target zone
// 			m_turnInput = 0;
// 			}
// 			// printf("Limelight Yaw: %.01f",yawRate);
// 			m_pDrivetrain->Drive(units::meters_per_second_t(robotXVel), // set the driveTrain
// 						units::meters_per_second_t(robotYVel),
// 						units::radians_per_second_t(-yawRate),
// 						false);
// 			frc::SmartDashboard::PutNumber("limelightYawRate", yawRate);

// 		}else{
// 			// update drive
// 		followerYaw = 0; //temp
// 		m_pDrivetrain->Drive(units::meters_per_second_t(robotXVel),
// 							 units::meters_per_second_t(robotYVel), 
// 							 units::degrees_per_second_t(-followerYaw), 
// 							 true
// 							);
// 		}
		
// 		frc::SmartDashboard::PutNumber("Odometry X", m_pDrivetrain->GetPose().Translation().X().to<double>());
//         frc::SmartDashboard::PutNumber("Odometry Y", m_pDrivetrain->GetPose().Translation().Y().to<double>());
//         frc::SmartDashboard::PutNumber("Odometry Yaw", m_pDrivetrain->GetPose().Rotation().Degrees().to<double>());

// 	}

// 	bool IsFinished() {
// 		return m_endTime - m_timer.Get() <= 0 || (m_distToEnd < m_targetZone) || m_lastPointReached;
// 	}

// 	void End(bool interrupted) {
// 		m_File.close();
// 		frc::SmartDashboard::PutBoolean("auto interrupted", interrupted);
// 		frc::SmartDashboard::PutBoolean("end auto", (m_distToEnd < m_targetZone) || m_lastPointReached);
// 		frc::SmartDashboard::PutBoolean("auto timed out", m_endTime - m_timer.Get() <= 0);
// 		m_timer.Stop();
// 		m_pDrivetrain->stop();
// 	}

//   //scp admin@10.24.81.2:/home/lvuser/SwervePath.csv  .
//   //
  

// private:
// 	bool m_tv =false;
// 	bool m_autoTargetEngaged = true;
// 	int m_pathIndex = 0;
// 	std::vector<SwerveDrivePathGenerator::finalPathPoint_t> m_path;
//   	DriveSubsystem* m_pDrivetrain;
// 	bool m_lastPointReached;
// 	double m_distToEnd = 0;
// 	double m_targetZone = 0;
// 	double m_time = 0;
// 	double m_endTime = 0;
// 	frc::Timer m_timer;
// 	std::ofstream m_File;
// 	frc2::PIDController m_drivePIDController{
//       1, 0, 0};
// 	frc::ProfiledPIDController<units::radians> m_turningPIDController{
// 		3, 0, 0.2,
// 		AutoConstants::kThetaControllerConstraints};
// };
