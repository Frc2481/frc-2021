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
// #include <frc/geometry/Pose2d.h>
// #include "Utils/CoordinateTranslation.h"
// #include <frc/controller/ProfiledPIDController.h>
// /**
//  * An example command.
//  *
//  * <p>Note that this extends CommandHelper, rather extending CommandBase
//  * directly; this is crucially important, or else the decorator functions in
//  * Command will *not* work!
//  */

// //0.3048 feet to meters
// class AutoSwerveFollowPathCommand
//     : public frc2::CommandHelper<frc2::CommandBase, AutoSwerveFollowPathCommand> {
//  public:
//   AutoSwerveFollowPathCommand(std::vector<SwerveDrivePathGenerator::finalPathPoint_t> &path,
// 		double targetZone,
//     	DriveSubsystem* driveTrain): m_lastPointReached(false),
// 		m_distToEnd(std::numeric_limits<double>::infinity()),
// 		m_targetZone(targetZone),
// 		m_time(0){

//     m_path = path;
// 	m_pDrivetrain = driveTrain;
//     AddRequirements(m_pDrivetrain);
// 	}
// 	void Initialize() {
// 		m_turningPIDController.EnableContinuousInput(-180, 180);
// 		// m_pDrivetrain->ZeroHeading();//starting at first point in the path
// 		m_pDrivetrain->ResetOdometry(frc::Pose2d(frc::Translation2d(units::meter_t(m_path[0].xPos), units::meter_t(m_path[0].yPos)),frc::Rotation2d(units::degree_t(m_path[0].yaw))));
// 		std::remove("home/lvuser/ActualPath.csv");
// 		m_File.open("home/lvuser/ActualPath.csv");
// 		m_File <<"time(s),  Actual Time (s), xPos (m), yPos (m), vel (deg), yaw (m/s), YawRate (deg/s)," << //7
// 				 "ActualXPos(m), ActualYPos(m), ActualYaw(deg), skipped, ActualXVel (m/s), ActualYVel(m/s), ActualYawRate(m/s), AlctualRobotVel (m/s),"<<//8
// 				 "look ahead x (m), look ahead y(m), look ahead vectorAngle, commanded x vel, commanded y vel,"<<//5
// 				 "fr angle, fl angle, br angle, bl angle,"<<//4
// 				 "generated time (s), generated Vel(m/s), generated xPos (m), generated yPos (m), generated yaw(r), generated yawRate(rps), generated heading\n";//7
// 		frc::SmartDashboard::PutBoolean("auto interrupted", false);
// 		frc::SmartDashboard::PutBoolean("end auto", false);
// 		frc::SmartDashboard::PutBoolean("auto timed out",false);
		
// 		frc::SmartDashboard::PutNumber("path generated time",m_path.end()->time + RobotParameters::k_pathFollowerTimeoutAllowance);
// 		// frc::SmartDashboard::PutNumber("start time", m_startTime);
// 		m_timer.Start();
	
// 		// frc::SmartDashboard::PutNumber("time", m_timer.Get());
// 		m_endTime = m_path.end()->time + RobotParameters::k_pathFollowerTimeoutAllowance+m_timer.Get();
// 		//for testing
// 		// m_timer.Stop();  
// 	}

// 	void Execute() {
// 		// frc::SmartDashboard::PutNumber("time", m_timer.Get());
// 		// update pose
// 		frc::Pose2d pose = m_pDrivetrain->GetPose();
// 		double robotAngle = pose.Rotation().Degrees().to<double>(); //m_pDrivetrain->GetHeading();
		
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
// 		for(int j = m_pathIndex; j < (int)m_path.size(); j++){
// 			frc::Translation2d vectPointToRobot = pose.Translation() - frc::Translation2d(units::meter_t(m_path[j].xPos),units::meter_t(m_path[j].yPos));
// 			double distToPoint = vectPointToRobot.Norm().to<double>();
// 			double dirToPointRad = atan2(-vectPointToRobot.Y().to<double>(),-vectPointToRobot.X().to<double>());
// 			dirToPointRad = normalizeToRange::NormalizeToRange(dirToPointRad+90,-MATH_CONSTANTS_PI, MATH_CONSTANTS_PI,false);
// 			double pointDirRad = m_path[j].heading*MATH_CONSTANTS_PI/180;
// 			// printf("dirToPointRad: %f, pointDirRad: %f\n", dirToPointRad, pointDirRad);
// 			frc::SmartDashboard::PutNumber("angleToPoint", fabs(dirToPointRad - pointDirRad));
// 			//find closest point that is a specific time ahead so you dont grab points to far ahead in the path
// 			diffAngle = fabs(normalizeToRange::RangedDifference(dirToPointRad - pointDirRad, -MATH_CONSTANTS_PI, MATH_CONSTANTS_PI));
// 			// diffAngle = fabs(dirToPointRad - pointDirRad);
// 			// if(MATH_CONSTANTS_PI < diffAngle){
// 			// 	diffAngle = 2*MATH_CONSTANTS_PI - diffAngle;
// 			// }
// 			if(distToPoint < distToClosestPoint){
// 				if(MATH_CONSTANTS_PI/2 > diffAngle && fabs(m_path[j].vel) > RobotParameters::k_minRobotVelocity){
// 					m_pathIndex = j;
// 					distToClosestPoint = distToPoint;
// 					driftingAway = 0;
// 				}else{
// 					skipped++;
// 				}
// 			}else{
// 				driftingAway++;
// 				if(driftingAway >= m_maxDrifting){
// 					break;
// 				}
// 			}
// 		}
// 		frc::SmartDashboard::PutNumber("skipped", skipped);
// 		frc::SmartDashboard::PutNumber("vel", m_path[m_pathIndex].vel);
// 		// check if last point reached
// 		if(m_pathIndex == (int)m_path.size()-1 || distToClosestPoint == std::numeric_limits<double>::infinity() ) {
// 			// printf("ended 120");
// 			frc::SmartDashboard::PutBoolean("path should stop", true);
// 			m_lastPointReached  = true;
// 		}
// 		frc::SmartDashboard::PutNumber("points left in the path", m_path.size() - m_pathIndex);
// 		// frc::SmartDashboard::PutNumber("distToClosestPoint", distToClosestPoint);
// 		for(int j = m_pathIndex; j < (int)m_path.size(); j++){
// 			frc::Translation2d vectPointToRobot = pose.Translation() - frc::Translation2d(units::meter_t(m_path[j].xPos),units::meter_t(m_path[j].yPos)); //frc::Translation2d(units::meter_t(m_path[j].xPos),units::meter_t(m_path[j].yPos))-  frc::Translation2d(units::meter_t(m_path[m_generatedPathIndex].xPos),units::meter_t(m_path[m_generatedPathIndex].yPos)); Robot centric
// 			double distToPoint = vectPointToRobot.Norm().to<double>();
// 			if(distToPoint < m_lookAhead){
// 				// frc::SmartDashboard::PutNumber("lookaheadPointFinding", distToPoint);
// 				m_lookAheadIndex = j;
// 			}else{
				
// 				break;
// 			}
// 		}
// // frc::SmartDashboard::PutNumber("look ahead point", (pose.Translation() - frc::Translation2d(units::meter_t(m_path[m_lookAheadIndex].xPos),units::meter_t(m_path[m_lookAheadIndex].yPos))).Norm().to<double>());
		
		
// 		frc::SmartDashboard::PutNumber("pathSpeed", m_path[m_pathIndex].vel);
// 		m_time += 1.0/RobotParameters::k_updateRate;

// 		double robotYawRate = m_path[m_pathIndex].yawRate;
// 		// double robotXVel = m_path[m_pathIndex].xVel;
// 		// double robotYVel = m_path[m_pathIndex].yVel;
// 		double robotVel = m_path[m_pathIndex].vel;
// 		double poseX = m_path[m_lookAheadIndex].xPos;
// 		double poseY = m_path[m_lookAheadIndex].yPos;
// 		double followerYaw = m_path[m_pathIndex].yaw;
		
// 		double closestX = m_path[m_pathIndex].xPos;
// 		double closestY = m_path[m_pathIndex].yPos;
// 		//calculating new vector
// 		double distX = poseX - pose.Translation().X().to<double>();
// 		double distY = poseY - pose.Translation().Y().to<double>();
// 		double vectorAngle = atan2(distY, distX);
// 		// update drive
// 		// followerYaw = 0; //temp
// 		//field centric driving
// 		double yawRate = m_turningPIDController.Calculate(robotAngle,followerYaw);
// 		// robotVel = RobotParameters::k_maxSpeed;//TODO remove
// 		m_pDrivetrain->Drive(units::meters_per_second_t(cos(vectorAngle)*robotVel),
// 							 units::meters_per_second_t(sin(vectorAngle)*robotVel), 
// 							 units::degrees_per_second_t(yawRate), 
// 							 true,
// 							 false
// 							);
// 		//driving
// 		// m_pDrivetrain->Drive(units::meters_per_second_t(RobotParameters::k_maxSpeed),
// 		// 					 units::meters_per_second_t(0), 
// 		// 					 units::degrees_per_second_t(0), 
// 		// 					 true,
// 		// 					 false
// 		// 					);
// 		frc::SmartDashboard::PutNumber("Odometry X", m_pDrivetrain->GetPose().Translation().X().to<double>());
//         frc::SmartDashboard::PutNumber("Odometry Y", m_pDrivetrain->GetPose().Translation().Y().to<double>());
//         frc::SmartDashboard::PutNumber("Odometry Yaw", m_pDrivetrain->GetPose().Rotation().Degrees().to<double>());
// 		m_generatedPathIndex++;

// 		// robotXVel = m_drivePIDController.Calculate(robotXVel);//,poseX - pose.Translation().X().to<double>());
// 		// robotYVel = m_drivePIDController.Calculate(robotYVel);//,poseY- pose.Translation().Y().to<double>());
		
// 		// if(fabs(robotYawRate) >= RobotParameters::k_rotateToAngleMaxYawRate){
// 		// 	robotYawRate = RobotParameters::k_rotateToAngleMaxYawRate;
// 		// }
// 		double actualYaw = pose.Rotation().Degrees().to<double>();
// 		double xVel = m_pDrivetrain->GetRobotVelocity().vx.to<double>();
// 		double yVel =m_pDrivetrain->GetRobotVelocity().vy.to<double>();
// 		m_File << m_path[m_pathIndex].time << ",";
// 		m_File << m_timer.Get() << ",";
// 		m_File << closestX *m_metersToInches<< ",";
// 		m_File << closestY*m_metersToInches << ",";
// 		m_File << robotVel*m_metersToInches << ",";
// 		m_File << followerYaw << ",";
// 		m_File << robotYawRate << ",";//7


// 		m_File << pose.Translation().X().to<double>()*m_metersToInches << ",";
// 		m_File << pose.Translation().Y().to<double>()*m_metersToInches << ",";
// 		m_File << actualYaw << ",";
// 		m_File << skipped << ",";//
// 		m_File << xVel *m_metersToInches<< ",";
// 		m_File << yVel *m_metersToInches<< ",";
// 		m_File << m_pDrivetrain->GetRobotVelocity().omega.to<double>() << ",";
// 		m_File << sqrt(xVel*xVel+yVel*yVel) *m_metersToInches<< ",";//8


// 		//look ahead info
// 		m_File << m_path[m_lookAheadIndex].xPos *m_metersToInches<< ",";
// 		m_File << m_path[m_lookAheadIndex].yPos *m_metersToInches<< ",";
// 		m_File << vectorAngle*180/MATH_CONSTANTS_PI << ",";
// 		m_File << cos(vectorAngle)*robotVel *m_metersToInches << ",";
// 		m_File << sin(vectorAngle)*robotVel *m_metersToInches<< ",";//5
// 		//get motor angles for the drive train
// 		m_File << m_pDrivetrain->getFrontRightMotor().angle.Degrees().to<double>() << ","
//          << m_pDrivetrain->getFrontLeftMotor().angle.Degrees().to<double>() <<","
//          << m_pDrivetrain->getBackRightMotor().angle.Degrees().to<double>() << ","
//          << m_pDrivetrain->getBackLeftMotor().angle.Degrees().to<double>() << ",";//4

// 		if(m_generatedPathIndex <= (int)m_path.size()){
// 			m_File << m_path[m_generatedPathIndex].time << ",";
// 			m_File << m_path[m_generatedPathIndex].vel << ",";
// 			m_File << m_path[m_generatedPathIndex].xPos << ",";
// 			m_File << m_path[m_generatedPathIndex].yPos << ",";
// 			m_File << m_path[m_generatedPathIndex].yaw << ",";
// 			m_File << m_path[m_generatedPathIndex].yawRate << ",";
// 			m_File << m_path[m_generatedPathIndex].heading << ",";//7
// 		}
// 		m_File << "\n";
// 		// frc::SmartDashboard::PutNumber("robotYVel follower", robotYVel);
// 		// frc::SmartDashboard::PutNumber("x vel follower", cos(vectorAngle)*robotVel);
// 		// frc::SmartDashboard::PutNumber("y vel follower", sin(vectorAngle)*robotVel);
// 		// frc::SmartDashboard::PutNumber("yaw rate follower", robotYawRate);
// 		// frc::SmartDashboard::PutBoolean("end auto", (m_distToEnd < m_targetZone) || m_lastPointReached);
// 		frc::SmartDashboard::PutNumber("x follower pos", poseX);
// 		frc::SmartDashboard::PutNumber("y follower pos", poseY);
// 		// frc::SmartDashboard::PutNumber("follower yaw", followerYaw);
// 		// frc::SmartDashboard::PutNumber("x vel", pose.Translation().Y().to<double>());
		
// 	}

// 	bool IsFinished() {
// 		return (m_distToEnd < m_targetZone) || m_lastPointReached; //m_endTime - m_timer.Get() <= 0 || 
// 	}

// 	void End(bool interrupted) {
// 		m_File.close();
// 		frc::SmartDashboard::PutBoolean("auto interrupted", interrupted);
// 		frc::SmartDashboard::PutBoolean("end auto", (m_distToEnd < m_targetZone) || m_lastPointReached);
// 		frc::SmartDashboard::PutBoolean("auto timed out", m_endTime - m_timer.Get() <= 0);
// 		m_timer.Stop();
// 		m_pDrivetrain->stop();
// 	}

// private:
// int m_metersToInches = 39.3701;
// 	int m_pathIndex = 0;
// 	std::vector<SwerveDrivePathGenerator::finalPathPoint_t> m_path;
//   	DriveSubsystem* m_pDrivetrain;
// 	bool m_lastPointReached;
// 	double m_distToEnd = 0;
// 	double m_targetZone = 0;
// 	double m_time = 0;
// 	double m_endTime = 0;
// 	int m_generatedPathIndex = 0;
// 	double m_lookAhead = 24 * 0.0254;
// 	int m_lookAheadIndex = 0;
// 	int m_maxDrifting = 5;
// 	frc::Timer m_timer;
// 	std::ofstream m_File;
// 	frc2::PIDController m_turningPIDController{
//       8, 0, 0};
// };

