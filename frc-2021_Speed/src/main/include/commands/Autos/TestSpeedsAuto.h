/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/InstantCommand.h>

#include "commands/pathCommands/PathFollowerCommand.h"


#include "commands/Drive/DriveOpenLoopCommand.h"

#include "commands/Drive/RotateWithMotionMagic.h"

#include "Utils/SwerveDrivePathFollower.h"
#include "commands/Intake/IntakesDefaultCommand.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include "commands/Drive/EngageBakeCommand.h"
#include "commands/Intake/IntakesDefaultCommand.h"
#include "commands/Intake/ReverseIntakesCommands.h"
#include <frc2/Timer.h>
class TestSpeedsAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 TestSpeedsAuto> {
 public:
  TestSpeedsAuto(DriveSubsystem* m_drive,
                      IntakeSubsystem* m_intake){

    const double metersToInches = 39.3701;
//     double curveSmall = 17;//17;                      //17//half of robot(12) + half of cone(2.5) + buffer(2.5)
//     double curveBig = 25;
//     double testSpeed = RobotParameters::k_maxSpeed*metersToInches*1;
// double tempCurve = 12;// todo make part of curve add to the other two cones
// const double radCurveC = 24;
    //  double distToAccel = (std::pow(RobotParameters::k_maxSpeed,2)/(2*RobotParameters::k_maxAccel)) * metersToInches;
    //  double distToDeccel = fabs((std::pow(RobotParameters::k_maxSpeed,2)/(2*RobotParameters::k_maxDeccel)) * metersToInches);

//speeds
    std::vector<SwerveDrivePathGenerator::waypoint_t> start;   //-90
    start.push_back(SwerveDrivePathGenerator::waypoint_t {0, 0, 0, 0, 0});//start
    start.push_back(SwerveDrivePathGenerator::waypoint_t {100, 0, 0, RobotParameters::k_maxSpeed * metersToInches, 0});
    start.push_back(SwerveDrivePathGenerator::waypoint_t {200, 0, 0, RobotParameters::k_maxSpeed * metersToInches, 0});
    start.push_back(SwerveDrivePathGenerator::waypoint_t {300, 0, 0, 0, 0});//pick up ball 4 and 5
//circle
    // start.push_back(SwerveDrivePathGenerator::waypoint_t {150 + curveSmall, 60 + curveSmall, 0, testSpeed, 0});//cone 1, point 1
    // start.push_back(SwerveDrivePathGenerator::waypoint_t {150 + curveBig, 60 - curveBig, 0, testSpeed, 0});//cone 1, point 2
    // start.push_back(SwerveDrivePathGenerator::waypoint_t {150 - curveBig, 60 - curveBig, 0, testSpeed, 0});//cone 1, point 3
    // start.push_back(SwerveDrivePathGenerator::waypoint_t {150 + curveSmall, 60 + curveSmall, 0, testSpeed, 0});//cone 1, point 4
//bounce
    // start.push_back(SwerveDrivePathGenerator::waypoint_t {0, 0, 0, testSpeed, 0});//cone 1, point 2
    // start.push_back(SwerveDrivePathGenerator::waypoint_t {30, 0, 0, testSpeed, 0});//cone 1, point 3
    // start.push_back(SwerveDrivePathGenerator::waypoint_t {60, 0, 0, testSpeed, 0});//cone 1, point 4
    // std::vector<SwerveDrivePathGenerator::waypoint_t> end;
    // end.push_back(SwerveDrivePathGenerator::waypoint_t {60, 0, 0, testSpeed, 0});//cone 1, point 2
    // end.push_back(SwerveDrivePathGenerator::waypoint_t {30, 0, 0, testSpeed, 0});//cone 1, point 3
    // end.push_back(SwerveDrivePathGenerator::waypoint_t {0, 0, 0, 0, 0});//cone 1, point 4
// const double curve = 20;
// const double radCurveC = 30;
// start.push_back(SwerveDrivePathGenerator::waypoint_t {-curve, -curve, 0, testSpeed, radCurveC});//cone 1, point 2
// start.push_back(SwerveDrivePathGenerator::waypoint_t {-curve, curve, 0, testSpeed, radCurveC});//cone 1, point 3
// start.push_back(SwerveDrivePathGenerator::waypoint_t {curve, curve, 0, testSpeed, radCurveC});//cone 1, point 4
// start.push_back(SwerveDrivePathGenerator::waypoint_t {curve, -curve, 0, testSpeed, radCurveC});//cone 1, point 4
// start.push_back(SwerveDrivePathGenerator::waypoint_t {-curve, -curve, 0, 0, radCurveC});//cone 1, point 2
// start.push_back(SwerveDrivePathGenerator::waypoint_t {-curve, curve, 0, testSpeed, radCurveC});//cone 1, point 3
// start.push_back(SwerveDrivePathGenerator::waypoint_t {curve, curve, 0, testSpeed, radCurveC});//cone 1, point 4
// start.push_back(SwerveDrivePathGenerator::waypoint_t {curve, -curve, 0, testSpeed, radCurveC});//cone 1, point 4
// start.push_back(SwerveDrivePathGenerator::waypoint_t {-curve, -curve, 0, testSpeed, radCurveC});//cone 1, point 2
// start.push_back(SwerveDrivePathGenerator::waypoint_t {-curve, curve, 0, 30, radCurveC});//cone 1, point 3
// start.push_back(SwerveDrivePathGenerator::waypoint_t {curve, curve, 0, 30, radCurveC});//cone 1, point 4
// start.push_back(SwerveDrivePathGenerator::waypoint_t {curve, -curve, 0, 30, radCurveC});//cone 1, point 4
// start.push_back(SwerveDrivePathGenerator::waypoint_t {-curve, -curve, 0, 0, radCurveC});//cone 1, point 2
    AddCommands(
        frc2::ParallelCommandGroup{
            PathFollowerCommand(m_drive, start, "path path" ,true,false),
            frc2::SequentialCommandGroup{
                IntakesDefaultCommand(m_intake),
                frc2::WaitCommand(.1_s),
                ReverseIntakesCommands(m_intake),
                frc2::WaitCommand(.1_s),
                IntakesDefaultCommand(m_intake),
                frc2::WaitCommand(.1_s),
                ReverseIntakesCommands(m_intake),
                frc2::WaitCommand(.1_s),
                IntakesDefaultCommand(m_intake),
                frc2::WaitCommand(.1_s),
                ReverseIntakesCommands(m_intake),
                frc2::WaitCommand(.1_s),
                IntakesDefaultCommand(m_intake),
                frc2::WaitCommand(.1_s),
                ToggleIntakeCommand(m_intake)
            }
            
        }
      
    );
  }
};
