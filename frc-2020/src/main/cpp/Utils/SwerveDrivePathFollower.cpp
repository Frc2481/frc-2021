/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Utils/SwerveDrivePathFollower.h"
#include "Utils/Interpolate.h"

SwerveDrivePathFollower::SwerveDrivePathFollower():
					m_lastPointReached(false),
		m_distToEnd(std::numeric_limits<double>::infinity()),
		m_targetZone(0.05),
		m_time(0){
		m_xVect.push_back(0);
	m_xVect.push_back(RobotParameters::k_maxSpeed);

	m_yVect.push_back(PathConstants::kMinLookAhead);
	m_yVect.push_back(PathConstants::kMaxLookAhead);
		}

void SwerveDrivePathFollower::start(){
	m_pathIndex = 0;
	m_time = 0;
	m_pathFinished = false;
	m_lookAheadIndex = 0;
	m_distToEnd = std::numeric_limits<double>::infinity();
}

// This method will be called once per scheduler run
void SwerveDrivePathFollower::Update(frc::Pose2d pose){
    double robotAngle = pose.Rotation().Degrees().to<double>(); //m_pDrivetrain->GetHeading();
		
		// find closest point on path
		if(m_path.empty()) {
			printf("empty-----------\n");
			m_lastPointReached = true;
			m_distToEnd = 0;
			return;
		}

		frc::Translation2d vectClosestPointToRobot;
		double distToClosestPoint = std::numeric_limits<double>::infinity();
		int driftingAway = 0;
		int skipped = 0;
		double diffAngle = 0;
		m_distToEnd = (pose.Translation() - frc::Translation2d(units::meter_t(m_path[m_path.size()-1].xPos),units::meter_t(m_path[m_path.size()-1].yPos))).Norm().to<double>();
		// printf("size of path: %d\n", (int)m_path.size());
		for(int j = m_pathIndex; j < (int)m_path.size(); j++){
			// printf("point looking at: %d\n", j);
			
			frc::Translation2d vectPointToRobot = pose.Translation() - frc::Translation2d(units::meter_t(m_path[j].xPos),units::meter_t(m_path[j].yPos));
			double distToPoint = vectPointToRobot.Norm().to<double>();
			double dirToPointRad = atan2(vectPointToRobot.Y().to<double>(), vectPointToRobot.X().to<double>());
			dirToPointRad = normalizeToRange::NormalizeToRange(dirToPointRad,-MATH_CONSTANTS_PI, MATH_CONSTANTS_PI,false);
			double pointDirRad = m_path[j].heading*MATH_CONSTANTS_PI/180;
			// printf("dirToPointRad: %f, pointDirRad: %f\n", dirToPointRad, pointDirRad);
			frc::SmartDashboard::PutNumber("angleToPoint", fabs(dirToPointRad - pointDirRad)*180/MATH_CONSTANTS_PI);
			//find closest point that is a specific time ahead so you dont grab points to far ahead in the path
			diffAngle = fabs(normalizeToRange::RangedDifference(dirToPointRad - pointDirRad, -MATH_CONSTANTS_PI, MATH_CONSTANTS_PI));
			diffAngle = 0;//TODO check if neeeded
			if(distToPoint < distToClosestPoint){
				if(MATH_CONSTANTS_PI/2 > diffAngle && fabs(m_path[j].vel) > RobotParameters::k_minRobotVelocity){
					m_pathIndex = j;
					distToClosestPoint = distToPoint;
					driftingAway = 0;
				}else{
					skipped++;
				}
			}else{
				driftingAway++;
				if(driftingAway >= m_maxDrifting){
					break;
				}
			}
			// printf("diffAnge: %f, distToPoint: %f, distToClosestPoint %f, m_path vel: %f, driftingAway: %d\n",diffAngle, distToPoint,distToClosestPoint,m_path[j].vel, driftingAway);
		}
		frc::SmartDashboard::PutNumber("skipped", skipped);
		frc::SmartDashboard::PutNumber("vel", m_path[m_pathIndex].vel);
		// check if last point reached
		if(m_pathIndex == (int)m_path.size()-1 || distToClosestPoint == std::numeric_limits<double>::infinity() ) {
			// printf("ended 120");
			frc::SmartDashboard::PutBoolean("path should stop", true);
			m_lastPointReached  = true;
		}
		frc::SmartDashboard::PutNumber("points left in the path", m_path.size() - m_pathIndex);
		m_lookAhead = interpolate::interp(m_xVect, m_yVect, fabs(m_path[m_pathIndex].vel), false);
		// printf("------------------lookAhead distance: %f------------------\n", m_lookAhead);
		// frc::SmartDashboard::PutNumber("distToClosestPoint", distToClosestPoint);
		for(int j = m_pathIndex; j < (int)m_path.size(); j++){
			frc::Translation2d vectPointToRobot = frc::Translation2d(units::meter_t(m_path[j].xPos),units::meter_t(m_path[j].yPos))-  frc::Translation2d(units::meter_t(m_path[m_pathIndex].xPos),units::meter_t(m_path[m_pathIndex].yPos));//pose.Translation() - frc::Translation2d(units::meter_t(m_path[j].xPos),units::meter_t(m_path[j].yPos));  Robot centric
			double distToPoint = vectPointToRobot.Norm().to<double>();
			if(distToPoint < m_lookAhead){
				// frc::SmartDashboard::PutNumber("lookaheadPointFinding", distToPoint);
				m_lookAheadIndex = j;
			}else{
				
				break;
			}
		}
// frc::SmartDashboard::PutNumber("look ahead point", (pose.Translation() - frc::Translation2d(units::meter_t(m_path[m_lookAheadIndex].xPos),units::meter_t(m_path[m_lookAheadIndex].yPos))).Norm().to<double>());
		
		
		frc::SmartDashboard::PutNumber("pathSpeed", m_path[m_pathIndex].vel);
		m_time += 1.0/RobotParameters::k_updateRate;

		// double robotXVel = m_path[m_pathIndex].xVel;
		// double robotYVel = m_path[m_pathIndex].yVel;
		double robotVel = m_path[m_pathIndex].vel;
		double poseX = m_path[m_lookAheadIndex].xPos;
		double poseY = m_path[m_lookAheadIndex].yPos;
		double followerYaw = m_path[m_pathIndex].yaw;
		double followerYawRate = m_path[m_pathIndex].yawRate;
		//calculating new vector
		double distX = poseX - pose.Translation().X().to<double>();
		double distY = poseY - pose.Translation().Y().to<double>();
		double vectorAngle = atan2(distY, distX);
		// update drive
		// followerYaw = 0; //temp
		//field centric driving
		m_yawRate = m_turningPIDController.Calculate(robotAngle,followerYaw)+followerYawRate;//add
        m_xVel = cos(vectorAngle)*robotVel;
		m_yVel = sin(vectorAngle)*robotVel;


		// printf("path x vel update method %f\n", m_xVel);
		frc::SmartDashboard::PutNumber("x follower pos", poseX);
		frc::SmartDashboard::PutNumber("y follower pos", poseY);	
}


int SwerveDrivePathFollower::closestPointIndex(){
    return m_pathIndex;
}
int SwerveDrivePathFollower::lookAheadIndex(){
    return m_lookAheadIndex;
}
bool SwerveDrivePathFollower::isPathFinished(){
	frc::SmartDashboard::PutBoolean("distance", (m_distToEnd < m_targetZone));
	frc::SmartDashboard::PutNumber("distance to end", m_distToEnd);
	frc::SmartDashboard::PutBoolean("last point reached", m_lastPointReached);
	// printf("util path finished %b\n", m_pathFinished);
	// printf("distance to end%f, last point reached%d\n", m_distToEnd, (int)m_lastPointReached);
    return (m_distToEnd < m_targetZone) || m_lastPointReached;
}

void SwerveDrivePathFollower::generatePath(std::vector<SwerveDrivePathGenerator::waypoint_t> &waypoints, const std::string &name){
    double metersToInches = 39.87;
	SwerveDrivePathGenerator pathGenerator(
		waypoints,//tempWaypoints,
		RobotParameters::k_updateRate,
		RobotParameters::k_wheelTrack*metersToInches,
		RobotParameters::k_wheelBase*metersToInches,
		RobotParameters::k_maxSpeed*metersToInches,
		RobotParameters::k_maxAccel*metersToInches,
		RobotParameters::k_maxDeccel*metersToInches,
		RobotParameters::k_maxCentripAccel*metersToInches
		);
	pathGenerator.setIsReverse(false);
	pathGenerator.generatePath();
	pathGenerator.setPathFilename("/home/lvuser/"+name+".csv");
	pathGenerator.writeTempPathToCSV();
	pathGenerator.writePathToCSV();
	pathGenerator.writeComboPathToCSV();

	// m_path = pathGenerator.getFinalPath();
	CoordinateTranslation::NolanToWPILib(pathGenerator.getFinalPath(), m_path);
	m_path.erase(m_path.begin());
    m_turningPIDController.EnableContinuousInput(-180, 180);
    // std::remove("home/lvuser/ActualPath"+name+".csv");
    // m_File.open("home/lvuser/ActualPath"+name+".csv",std::ofstream::out);
    // m_File <<"time(s), xPos (m), yPos (m), vel (deg), yaw (m/s), YawRate (deg/s)," << //7
    //             "ActualXPos(m), ActualYPos(m), ActualYaw(deg), skipped,"<<//8
    //             "look ahead x (m), look ahead y(m), look ahead vectorAngle, commanded x vel, commanded y vel, look ahead (m)\n";//5
    
    
    frc::SmartDashboard::PutBoolean("auto interrupted", false);
    frc::SmartDashboard::PutBoolean("end auto", false);
		
}
double SwerveDrivePathFollower::getXVel(){
    return m_xVel;
}
double SwerveDrivePathFollower::getYVel(){
    return m_yVel;
}
double SwerveDrivePathFollower::getYawRate(){
    return m_yawRate;
}
frc::Pose2d SwerveDrivePathFollower::getPointPos(int index){
	return frc::Pose2d(units::meter_t(m_path[index].xPos),units::meter_t(m_path[index].yPos), frc::Rotation2d(units::degree_t(m_path[index].yaw)));
}
frc::Pose2d SwerveDrivePathFollower::getFollowerPos(){
	return frc::Pose2d(units::meter_t(m_path[m_pathIndex].xPos),units::meter_t(m_path[m_pathIndex].yPos), frc::Rotation2d(units::degree_t(m_path[m_pathIndex].yaw)));
}
void SwerveDrivePathFollower::stop(bool interrupted){
    // m_File.close();
    frc::SmartDashboard::PutBoolean("auto interrupted", interrupted);
    frc::SmartDashboard::PutBoolean("end auto", m_pathFinished);
}




