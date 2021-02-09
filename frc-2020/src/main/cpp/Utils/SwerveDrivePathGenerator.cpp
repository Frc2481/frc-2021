#include "Utils/SwerveDrivePathGenerator.h"
#include <limits>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <cmath>
#include "Utils/Interpolate.h"
#include "Utils/MathConstants.h"
#include "Utils/NormalizeToRange.h"
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include "Utils/Sign.h"
#include <math.h>
#include "Utils/IsDoubleEqual.h"

SwerveDrivePathGenerator::SwerveDrivePathGenerator(
    std::vector<waypoint_t> &waypoints,
    unsigned sampleRate,
    double wheelTrack,
	double wheelBase,
    double maxSpeed,
    double maxAccel,
    double maxDeccel,
    double maxCentripAccel)

    : m_tempPath(),
    m_comboPath(),
    m_finalPath(),
    m_sampleRate(sampleRate),
    m_isReverse(false),
    m_wheelTrack(wheelTrack),
	m_wheelBase(wheelBase),
    m_maxSpeed(maxSpeed),
    m_maxAccel(maxAccel),
    m_maxDeccel(maxDeccel),
    m_maxCentripAccel(maxCentripAccel),
    m_waypointsFilename("SwervePathGeneratorWaypoints.csv"),
    m_pathFilename("SwervePathGeneratorFinalPath.csv") {

    setWaypoints(waypoints);
}

SwerveDrivePathGenerator::~SwerveDrivePathGenerator() {
}

bool roughlyEqual(double a,  double b){
    return fabs(a-b) < 1e-6;
}
void SwerveDrivePathGenerator::setWaypoints(std::vector<waypoint_t> &waypoints) {
    m_waypoints.clear();
    for(std::vector<waypoint_t>::iterator it = waypoints.begin(); it != waypoints.end(); ++it) {
        it->speed = abs(it->speed);
        it->radCurve = abs(it->radCurve);
        m_waypoints.push_back(*it);
    }
}

void SwerveDrivePathGenerator::setSampleRate(unsigned sampleRate) {
    m_sampleRate = sampleRate;
}

void SwerveDrivePathGenerator::setIsReverse(bool isReverse) {
    m_isReverse = isReverse;
}

void SwerveDrivePathGenerator::setWheelTrack(double wheelTrack) {
    if(wheelTrack != 0) {
        m_wheelTrack = abs(wheelTrack);
    }
}

void SwerveDrivePathGenerator::setWheelBase(double wheelBase) {
    if(wheelBase != 0) {
        m_wheelBase = abs(wheelBase);
    }
}

void SwerveDrivePathGenerator::setMaxSpeed(double maxSpeed) {
    if(maxSpeed != 0) {
        m_maxSpeed = abs(maxSpeed);
    }
}

void SwerveDrivePathGenerator::setMaxAccel(double maxAccel) {
    if(maxAccel != 0) {
        m_maxAccel = abs(maxAccel);
    }
}

void SwerveDrivePathGenerator::setMaxDeccel(double maxDeccel) {
    if(maxDeccel != 0) {
        m_maxDeccel = -abs(maxDeccel);
    }
}

void SwerveDrivePathGenerator::setMaxCentripAccel(double maxCentripAccel) {
    if(maxCentripAccel != 0) {
        m_maxCentripAccel = abs(maxCentripAccel);
    }
}

void SwerveDrivePathGenerator::setWaypointsFilename(const std::string &waypointsFilename) {
    m_waypointsFilename = waypointsFilename;
}

void SwerveDrivePathGenerator::setPathFilename(const std::string &pathFilename) {
    m_pathFilename = pathFilename;
}

std::vector<SwerveDrivePathGenerator::finalPathPoint_t> SwerveDrivePathGenerator::getFinalPath() const {
    return m_finalPath;
}

double SwerveDrivePathGenerator::cross(frc::Translation2d translation, frc::Translation2d other) {
    return translation.X().to<double>() * other.Y().to<double>() - translation.Y().to<double>() * other.X().to<double>();
}

void SwerveDrivePathGenerator::generatePath() {
    pathGenPoint_t tempPathGenPoint;
	finalPathPoint_t tempComboPathPoint;
    finalPathPoint_t tempFinalPathPoint;

    // clear old path
    m_tempPath.clear();
    m_comboPath.clear();
    m_finalPath.clear();

    // add start point to path
    tempPathGenPoint.xPos = m_waypoints.front().xPos;
    tempPathGenPoint.yPos = m_waypoints.front().yPos;
    tempPathGenPoint.radCurve = std::numeric_limits<double>::infinity();
    tempPathGenPoint.vel = m_waypoints.front().speed;
    tempPathGenPoint.dist = 0;
    tempPathGenPoint.yaw = m_waypoints.front().yaw;
    m_tempPath.push_back(tempPathGenPoint);

    // generate path trajectory
    for(int i = 0; i < ((int)m_waypoints.size() - 2); ++i) {
        // get waypoint max distance threshold and speed
        double arcRad = m_waypoints[i + 1].radCurve;
        double speed = m_waypoints[i + 1].speed;
        double yaw = m_waypoints[i + 1].yaw;

        // get 3 consecutive points
        frc::Translation2d p1(units::meter_t(m_waypoints[i].xPos), units::meter_t(m_waypoints[i].yPos)); // current waypoint
        frc::Translation2d p2(units::meter_t(m_waypoints[i + 1].xPos), units::meter_t(m_waypoints[i + 1].yPos)); // next waypoint
        frc::Translation2d p3(units::meter_t(m_waypoints[i + 2].xPos), units::meter_t(m_waypoints[i + 2].yPos)); // next next waypoint

        // get vectors between points
        frc::Translation2d v21 = p1 - p2;
        frc::Translation2d v23 = p3 - p2;

        // check if redundant points
        double theta;
        if(isDoubleEqual::isDoubleEqual(v21.Norm().to<double>(), 0, 0.001) || isDoubleEqual::isDoubleEqual(v23.Norm().to<double>(), 0, 0.001)) {
            continue;    // else skip waypoint
        }
        else {
            theta = safeACos((v21.X().to<double>() * v23.X().to<double>()+v21.Y().to<double>() * v23.Y().to<double>()) / (v21.Norm().to<double>() * v23.Norm().to<double>())); // angle between v21 and v23
        }

        // check if points lie on same line
        if(isDoubleEqual::isDoubleEqual(theta, MATH_CONSTANTS_PI, 0.00002) || isDoubleEqual::isDoubleEqual(theta, 0, 0.00002)) {
            // do not insert rounded corner
            tempPathGenPoint.xPos = p2.X().to<double>();
            tempPathGenPoint.yPos = p2.Y().to<double>();
            tempPathGenPoint.radCurve = std::numeric_limits<double>::infinity();
            tempPathGenPoint.vel = speed;
            tempPathGenPoint.yaw = yaw;
            m_tempPath.push_back(tempPathGenPoint);
        }
        else {
            // calculate arc between points
            double arcCenterDist = arcRad * (1 - sin(theta / 2.0)) / sin(theta / 2.0);
            if(arcRad < (m_wheelTrack / 2.0)) { // remove velocity spike at discontinuity
                arcRad = (m_wheelTrack / 2.0);
            }
            double arcHeight = arcRad * (1 - cos((MATH_CONSTANTS_PI - theta) / 2.0));
            double arcChordLen = 2 * arcRad * sin((MATH_CONSTANTS_PI - theta) / 2.0);
            double straightDist = sqrt(pow((arcCenterDist + arcHeight), 2) + pow((arcChordLen / 2.0), 2));
                // distance between p2 and point where path transitions from straight to rounded corner

            // limit arc radius if too large for distance between points
            double limitDist = std::min(v21.Norm().to<double>(), v23.Norm().to<double>()) / 2.0;
            if(straightDist > limitDist) {
                straightDist = limitDist;
                arcChordLen = 2 * straightDist * sin(theta / 2.0);
                arcRad = arcChordLen * sin(theta / 2.0) / sin(MATH_CONSTANTS_PI - theta);
                arcHeight = arcRad * (1 - cos((MATH_CONSTANTS_PI - theta) / 2.0));
                arcCenterDist = straightDist * cos(theta / 2.0) - arcHeight;
            }

            // calculate center point of arc
            frc::Translation2d v26 = (v21 * (1 / v21.Norm().to<double>()) + v23 * (1 / v23.Norm().to<double>())) * 0.5;
            frc::Translation2d p6 = p2 + v26 * ((arcCenterDist + arcRad) / v26.Norm().to<double>()); // center point of arc

            // generate points along arc
            double phiStep = (MATH_CONSTANTS_PI - theta) / SWERVE_NUM_PHI_STEPS;
            double phi = Sign::Sign(cross(v21,v23)) * (MATH_CONSTANTS_PI - theta) / 2.0;
            for(int j = 1; j <= SWERVE_NUM_PHI_STEPS; j++) {
                frc::Translation2d p7 = p6 - v26.RotateBy(frc::Rotation2d(units::radian_t(phi))) * (arcRad / v26.Norm().to<double>());
                tempPathGenPoint.xPos = p7.X().to<double>();
                tempPathGenPoint.yPos = p7.Y().to<double>();

                // check if arc endpoint
                if((j == 1) || (j == SWERVE_NUM_PHI_STEPS)) {
                    tempPathGenPoint.radCurve = std::numeric_limits<double>::infinity();
                }
                else {
                    tempPathGenPoint.radCurve = arcRad;
                }

                // check if waypoint
                if(j == ceil(SWERVE_NUM_PHI_STEPS / 2.0)) {
                    tempPathGenPoint.vel = speed;
                    tempPathGenPoint.yaw = yaw;
                }
                else {
                    tempPathGenPoint.vel = std::max(speed, m_waypoints[i].speed);
                    tempPathGenPoint.yaw = std::numeric_limits<double>::infinity();
                }

                // add point to path gen points and increment phi
                m_tempPath.push_back(tempPathGenPoint);
                phi = phi - Sign::Sign(cross(v21, v23)) * phiStep;
            }
        }
    }

    // add end point to path
    tempPathGenPoint.xPos = m_waypoints.back().xPos;
    tempPathGenPoint.yPos = m_waypoints.back().yPos;
    tempPathGenPoint.radCurve = std::numeric_limits<double>::infinity();
    tempPathGenPoint.vel = m_waypoints.back().speed;
    tempPathGenPoint.yaw = m_waypoints.back().yaw;
    m_tempPath.push_back(tempPathGenPoint);

    // calculate distance traveled along path
    std::vector<double> tempPathDist;
    std::vector<double> tempPathXPos;
    std::vector<double> tempPathYPos;
    std::vector<double> tempPathYaw;
    m_totalPathDist = 0;
    m_tempPath.front().dist = m_totalPathDist;
    tempPathDist.push_back(m_tempPath.front().dist);
    tempPathXPos.push_back(m_tempPath.front().xPos);
    tempPathYPos.push_back(m_tempPath.front().yPos);
    tempPathYaw.push_back(m_tempPath.front().yaw);
    for(int i = 1; i < (int)m_tempPath.size(); ++i) {
        frc::Translation2d v21 = frc::Translation2d(units::meter_t(m_tempPath[i].xPos), units::meter_t(m_tempPath[i].yPos))
            - frc::Translation2d(units::meter_t(m_tempPath[i - 1].xPos), units::meter_t(m_tempPath[i - 1].yPos));
        m_totalPathDist += v21.Norm().to<double>();
        m_tempPath[i].dist = m_totalPathDist;

        // store temp path dist, xPos, yPos, yaw in separate vectors
        tempPathDist.push_back(m_tempPath[i].dist);
        tempPathXPos.push_back(m_tempPath[i].xPos);
        tempPathYPos.push_back(m_tempPath[i].yPos);
        tempPathYaw.push_back(m_tempPath[i].yaw);
    }

    // integrate path forward
    std::vector<pathGenPoint_t> fwdPath;
    integratePath(fwdPath, false);

    // integrate path backward
    std::vector<pathGenPoint_t> bwdPath;
    integratePath(bwdPath, true);

    // combine forward and backward paths with min speed
    for(int i = 0; i < (int)fwdPath.size(); ++i) {
        tempComboPathPoint.dist = fwdPath[i].dist;
        if(!m_isReverse) {
            tempComboPathPoint.vel = std::min(fwdPath[i].vel, bwdPath[i].vel);
        }
        else {
            tempComboPathPoint.vel = std::max(fwdPath[i].vel, bwdPath[i].vel);
        }

        m_comboPath.push_back(tempComboPathPoint);
    }

    // calculate path with respect to time
    std::vector<double> comboPathTime;
    std::vector<double> comboPathDist;
    std::vector<double> comboPathVel;
    m_comboPath.front().time = 0;
    comboPathTime.push_back(m_comboPath.front().time);
    comboPathDist.push_back(m_comboPath.front().dist);
    comboPathVel.push_back(m_comboPath.front().vel);
    for(int i = 1; i < (int)m_comboPath.size(); ++i) {
        // check divide by zero
        if((m_comboPath[i].vel + m_comboPath[i - 1].vel) != 0) {
            m_comboPath[i].time = m_comboPath[i - 1].time
                + 2 * (m_comboPath[i].dist - m_comboPath[i - 1].dist) / abs(m_comboPath[i].vel + m_comboPath[i - 1].vel);
        }
        else {
            m_comboPath[i].time = m_comboPath[i - 1].time;
        }

        // store combo path time, dist, vel in separate vectors
        comboPathTime.push_back(m_comboPath[i].time);
        comboPathDist.push_back(m_comboPath[i].dist);
        comboPathVel.push_back(m_comboPath[i].vel);
    }

    // calculate path using only valid yaw points
    std::vector<double> tempPathYawValidDist;
    std::vector<double> tempPathYawValidYaw;
    for(int i = 0; i < (int)m_tempPath.size(); ++i) {
        if(m_tempPath[i].yaw != std::numeric_limits<double>::infinity()) {
            tempPathYawValidDist.push_back(m_tempPath[i].dist);
            tempPathYawValidYaw.push_back(m_tempPath[i].yaw);
        }
    }

    // add first point to final path
    tempFinalPathPoint.time = 0;
    tempFinalPathPoint.dist = 0;
    tempFinalPathPoint.xPos = m_tempPath.front().xPos;
    tempFinalPathPoint.yPos = m_tempPath.front().yPos;
    tempFinalPathPoint.yaw = m_tempPath.front().yaw;
    tempFinalPathPoint.vel = m_tempPath.front().vel;
    tempFinalPathPoint.accel = m_maxAccel * (1 - fabs(tempFinalPathPoint.vel) / m_maxSpeed);
	if(m_isReverse) {
		tempFinalPathPoint.accel *= -1;;
	}
    tempFinalPathPoint.yawRate = 0;
    m_finalPath.push_back(tempFinalPathPoint);
	double prevVel = tempFinalPathPoint.vel;

    // calculate final path
    while(tempFinalPathPoint.time <= m_comboPath.back().time) {
        tempFinalPathPoint.time += 1 / (double)m_sampleRate;
        tempFinalPathPoint.dist = interpolate::interp(comboPathTime, comboPathDist, tempFinalPathPoint.time, false);
        tempFinalPathPoint.xPos = interpolate::interp(tempPathDist, tempPathXPos, tempFinalPathPoint.dist, false);
        tempFinalPathPoint.yPos = interpolate::interp(tempPathDist, tempPathYPos, tempFinalPathPoint.dist, false);
        tempFinalPathPoint.yaw = interpolate::rangedInterp(tempPathYawValidDist, tempPathYawValidYaw, tempFinalPathPoint.dist, false, -180, 180);
        double dx = tempFinalPathPoint.xPos - m_finalPath.back().xPos;
        double dy = tempFinalPathPoint.yPos - m_finalPath.back().yPos;
        frc::Rotation2d heading(dx, dy);
        tempFinalPathPoint.heading = heading.Degrees().to<double>();
        tempFinalPathPoint.vel = interpolate::interp(comboPathTime, comboPathVel, tempFinalPathPoint.time, false);
        tempFinalPathPoint.accel = (tempFinalPathPoint.vel - prevVel) * m_sampleRate;
        tempFinalPathPoint.yawRate = normalizeToRange::RangedDifference(tempFinalPathPoint.yaw - m_finalPath.back().yaw, -180, 180) * m_sampleRate;

        // add heading to first point in final path	
        if(m_finalPath.size() == 1) {
            m_finalPath.front().heading = tempFinalPathPoint.heading;
        }

        m_finalPath.push_back(tempFinalPathPoint);
		prevVel = tempFinalPathPoint.vel;
    }
}

void SwerveDrivePathGenerator::writePathToCSV() const {
    std::remove(m_pathFilename.c_str());

    std::ofstream myFile;
    myFile.open(m_pathFilename.c_str());
    myFile << "time (s), dist (in), xPos (in), yPos (in), yaw (deg), heading (deg), vel (in/s), accel (in/s^2), yawRate (deg/s)\n";

    for(int i = 0; i < (int)m_finalPath.size(); ++i) {
        myFile << m_finalPath[i].time << ",";
        myFile << m_finalPath[i].dist << ",";
        myFile << m_finalPath[i].xPos << ",";
        myFile << m_finalPath[i].yPos << ",";
        myFile << m_finalPath[i].yaw << ",";
        myFile << m_finalPath[i].heading << ",";
        myFile << m_finalPath[i].vel << ",";
        myFile << m_finalPath[i].accel << ",";
        myFile << m_finalPath[i].yawRate;
        myFile << "\n";
    }

    myFile.close();
}

void SwerveDrivePathGenerator::writeTempPathToCSV() const {
    std::remove("SwervePathGeneratorTempPath.csv");

    std::ofstream myFile;
    myFile.open("SwervePathGeneratorTempPath.csv");
    myFile << "dist (in), xPos (in), yPos (in), yaw (deg), vel (in/s), radCurve (in)\n";

    for(int i = 0; i < (int)m_tempPath.size(); ++i) {
        myFile << m_tempPath[i].dist << ",";
        myFile << m_tempPath[i].xPos << ",";
        myFile << m_tempPath[i].yPos << ",";
		myFile << m_tempPath[i].yaw << ",";
        myFile << m_tempPath[i].vel << ",";
        myFile << m_tempPath[i].radCurve;
        myFile << "\n";
    }

    myFile.close();
}

void SwerveDrivePathGenerator::writeComboPathToCSV() const {
    std::remove("SwervePathGeneratorTempComboPath.csv");

    std::ofstream myFile;
    myFile.open("SwervePathGeneratorTempComboPath.csv");
    myFile << "time (s), dist (in), xPos (in), yPos (in), yaw (deg), heading (deg), vel (in/s), accel (in/s^2), yawRate (deg/s)\n";

    for(int i = 0; i < (int)m_comboPath.size(); ++i) {
        myFile << m_comboPath[i].time << ",";
        myFile << m_comboPath[i].dist << ",";
        myFile << m_comboPath[i].xPos << ",";
        myFile << m_comboPath[i].yPos << ",";
        myFile << m_comboPath[i].yaw << ",";
        myFile << m_comboPath[i].heading << ",";
        myFile << m_comboPath[i].vel << ",";
        myFile << m_comboPath[i].accel << ",";
        myFile << m_comboPath[i].yawRate;
        myFile << "\n";
    }

    myFile.close();
}

void SwerveDrivePathGenerator::readWaypointsFromCSV() {
    std::vector<waypoint_t> waypoints;
    waypoint_t tempWaypoint;
    std::string header;
    char delim;

    // open file
    std::ifstream myFile;
    myFile.open(m_waypointsFilename.c_str(), std::ifstream::in);

    // read config
    unsigned sampleRate;
    getline(myFile, header, ',');
    myFile >> sampleRate;
    getline(myFile, header); // skip extra delimeters
    setSampleRate(sampleRate);

    bool isReverse;
    getline(myFile, header, ',');
    myFile >> isReverse;
    getline(myFile, header); // skip extra delimeters
    setIsReverse(isReverse);

    double wheelTrack;
    getline(myFile, header, ',');
    myFile >> wheelTrack;
    getline(myFile, header); // skip extra delimeters
    setWheelTrack(wheelTrack);

    double maxSpeed;
    getline(myFile, header, ',');
    myFile >> maxSpeed;
    getline(myFile, header); // skip extra delimeters
    setMaxSpeed(maxSpeed);

    double maxAccel;
    getline(myFile, header, ',');
    myFile >> maxAccel;
    getline(myFile, header); // skip extra delimeters
    setMaxAccel(maxAccel);

    double maxDeccel;
    getline(myFile, header, ',');
    myFile >> maxDeccel;
    getline(myFile, header); // skip extra delimeters
    setMaxDeccel(maxDeccel);

    double maxCentripAccel;
    getline(myFile, header, ',');
    myFile >> maxCentripAccel;
    getline(myFile, header); // skip extra delimeters
    setMaxCentripAccel(maxCentripAccel);

    // read data
    getline(myFile, header); // skip blank line
    getline(myFile, header); // skip headers

    while(myFile >> tempWaypoint.xPos >> delim
                >> tempWaypoint.yPos >> delim
                >> tempWaypoint.speed >> delim
				>> tempWaypoint.yaw >> delim
                >> tempWaypoint.radCurve) {
        waypoints.push_back(tempWaypoint);
        getline(myFile, header); // skip extra delimeters
    }

    // close file
    myFile.close();

    // set waypoints
    setWaypoints(waypoints);
}

void SwerveDrivePathGenerator::integratePath(std::vector<pathGenPoint_t> &integratedPath, bool isBackward) {
    pathGenPoint_t tempPathGenPoint;

    // clear path
    integratedPath.clear();

    // add start point to path
    if(!isBackward) {
        tempPathGenPoint.dist = 0;
        tempPathGenPoint.vel = m_tempPath.front().vel;
    }
    else {
        tempPathGenPoint.dist = m_totalPathDist;
        tempPathGenPoint.vel = m_tempPath.back().vel;
    }

	if(m_isReverse) {
		tempPathGenPoint.vel *= -1;
	}

    integratedPath.push_back(tempPathGenPoint);

    // integrate path
    double accelSpeed;
    double pathSpeed;
    double latSlipSpeed;
    double limitWheelSpeed;
	double radiusCurve;
    int i;
	int j;
	double deltaDist;
	double deltaYaw;
	double leverArm;

    if(!isBackward) {
        i = 1;
		j = 1;
        while(true) {
			if(m_tempPath[j].yaw != std::numeric_limits<double>::infinity()) {
				break;
			}
			j++;
		}

		deltaDist = fabs(m_tempPath[j].dist - m_tempPath.front().dist);
		deltaYaw = fabs(normalizeToRange::RangedDifference(m_tempPath[j].yaw - m_tempPath.front().yaw, -180.0, 180.0) * MATH_CONSTANTS_PI / 180.0);
		leverArm = sqrt(pow(m_wheelTrack / 2.0, 2) + pow(m_wheelBase / 2.0, 2));
		if((deltaDist + deltaYaw * leverArm) != 0) {
		    limitWheelSpeed = fabs(deltaDist * m_maxSpeed / (deltaDist + deltaYaw * leverArm));
        }
        else {
            limitWheelSpeed = std::numeric_limits<double>::infinity();
        }
    }
    else {
        i = m_tempPath.size() - 2;
		j = i;
        while(true) {
			if(m_tempPath[j].yaw != std::numeric_limits<double>::infinity()) {
				break;
			}
			j--;
		}

		deltaDist = fabs(m_tempPath[j].dist - m_tempPath.back().dist);
		deltaYaw = fabs(normalizeToRange::RangedDifference(m_tempPath[j].yaw - m_tempPath.back().yaw, -180.0, 180.0) * MATH_CONSTANTS_PI / 180.0);
		leverArm = sqrt(pow(m_wheelTrack / 2.0, 2) + pow(m_wheelBase / 2.0, 2));
        if((deltaDist + deltaYaw * leverArm) != 0) {
		    limitWheelSpeed = fabs(deltaDist * m_maxSpeed / (deltaDist + deltaYaw * leverArm));
        }
        else {
            limitWheelSpeed = std::numeric_limits<double>::infinity();
        }
    }

    while(((integratedPath.back().dist <= (m_totalPathDist - SWERVE_INTEGRATE_PATH_DIST_STEP)) && !isBackward)
          || ((integratedPath.back().dist >= SWERVE_INTEGRATE_PATH_DIST_STEP) && isBackward)) {
        // increment distance traveled and add to point
        if(!isBackward) {
            tempPathGenPoint.dist += SWERVE_INTEGRATE_PATH_DIST_STEP;
        }
        else {
            tempPathGenPoint.dist -= SWERVE_INTEGRATE_PATH_DIST_STEP;
        }

        // get path speed and limit individual wheel speed
        // assume that sample rate is high enough so that temp path points do not need skipped
        if(!isBackward) {
            pathSpeed = std::max(m_tempPath[i].vel, m_tempPath[i - 1].vel);

            if((tempPathGenPoint.dist + SWERVE_INTEGRATE_PATH_DIST_STEP) > m_tempPath[i].dist) {
                pathSpeed = m_tempPath[i].vel;
                i++;
                if(i >= (int)m_tempPath.size()) {
                    i = (int)m_tempPath.size() - 1;
                }

				if(m_tempPath[i].yaw != std::numeric_limits<double>::infinity()) {
					j = i + 1;
                    if(j >= (int)m_tempPath.size()) {
                        j = (int)m_tempPath.size() - 1;
                    }
                    
					while(true) {
						if(m_tempPath[j].yaw != std::numeric_limits<double>::infinity()) {
							break;
						}

						j++;
                        if(j >= (int)m_tempPath.size()) {
                            j = (int)m_tempPath.size() - 1;
                        }
					}

					deltaDist = fabs(m_tempPath[j].dist - m_tempPath[i].dist);
					deltaYaw = fabs(normalizeToRange::RangedDifference(m_tempPath[j].yaw - m_tempPath[i].yaw, -180.0, 180.0) * MATH_CONSTANTS_PI / 180.0);
					leverArm = sqrt(pow(m_wheelTrack / 2.0, 2) + pow(m_wheelBase / 2.0, 2));
					if((deltaDist + deltaYaw * leverArm) != 0) {
		                limitWheelSpeed = fabs(deltaDist * m_maxSpeed / (deltaDist + deltaYaw * leverArm));
                    }
                    else {
                        limitWheelSpeed = std::numeric_limits<double>::infinity();
                    }
				}
            }
        }
        else {
            pathSpeed = std::max(m_tempPath[i].vel, m_tempPath[i + 1].vel);

            if((tempPathGenPoint.dist - SWERVE_INTEGRATE_PATH_DIST_STEP) < m_tempPath[i].dist) {
                pathSpeed = m_tempPath[i].vel;
                i--;
                if(i < 0) {
                    i = 0;
                }

				if(m_tempPath[i].yaw != std::numeric_limits<double>::infinity()) {
					j = i - 1;
                    if(j < 0) {
                        j = 0;
                    }

					while(true) {
						if(m_tempPath[j].yaw != std::numeric_limits<double>::infinity()) {
							break;
						}

						j--;
                        if(j < 0) {
                            j = 0;
                        }
					}

					deltaDist = fabs(m_tempPath[j].dist - m_tempPath[i].dist);
					deltaYaw = fabs(normalizeToRange::RangedDifference(m_tempPath[j].yaw - m_tempPath[i].yaw, -180.0, 180.0) * MATH_CONSTANTS_PI / 180.0);
					leverArm = sqrt(pow(m_wheelTrack / 2.0, 2) + pow(m_wheelBase / 2.0, 2));
					if((deltaDist + deltaYaw * leverArm) != 0) {
		                limitWheelSpeed = fabs(deltaDist * m_maxSpeed / (deltaDist + deltaYaw * leverArm));
                    }
                    else {
                        limitWheelSpeed = std::numeric_limits<double>::infinity();
                    }
				}
            }
        }

        // calculate acceleration speed
        if(!isBackward) {
            accelSpeed = sqrt(pow(tempPathGenPoint.vel, 2) + 2.0 * m_maxAccel * (1 - fabs(tempPathGenPoint.vel) / m_maxSpeed) * SWERVE_INTEGRATE_PATH_DIST_STEP);
        }
        else {
            accelSpeed = sqrt(pow(tempPathGenPoint.vel, 2) - 2.0 * m_maxDeccel * SWERVE_INTEGRATE_PATH_DIST_STEP);
        }

        // hold and limit lateral slip speed through turn
        radiusCurve = std::numeric_limits<double>::infinity();
        if((1 < i) && (i < ((int)m_tempPath.size() - 2))) {
            if(!isBackward) {
                // check if in turn
                if(isDoubleEqual::isDoubleEqual(m_tempPath[i - 1].radCurve, m_tempPath[i].radCurve, 0.001)
                    && (m_tempPath[i].radCurve != std::numeric_limits<double>::infinity())) {
                    radiusCurve = m_tempPath[i].radCurve;
                }
            }
            else {
                // check if in turn
                if(isDoubleEqual::isDoubleEqual(m_tempPath[i + 1].radCurve, m_tempPath[i].radCurve, 0.001)
                    && (m_tempPath[i].radCurve != std::numeric_limits<double>::infinity())) {
                    radiusCurve = m_tempPath[i].radCurve;
                }
            }
        }

        latSlipSpeed = sqrt(m_maxCentripAccel * radiusCurve);

        // use minimum speed from all constraints
        tempPathGenPoint.vel = std::min(std::min(std::min(std::min(accelSpeed, pathSpeed), latSlipSpeed), limitWheelSpeed), m_maxSpeed);

        // reverse path direction if needed
        if(m_isReverse) {
        	tempPathGenPoint.vel *= -1;
        }

        // add to path
        integratedPath.push_back(tempPathGenPoint);
    }

    if(isBackward) {
        std::reverse(integratedPath.begin(), integratedPath.end());
    }
}

double SwerveDrivePathGenerator::safeACos(double val) const {
    if(val > 1) {
        val = 1;
    }
    else if (val < -1) {
        val = -1;
    }

    return acos(val);
}
