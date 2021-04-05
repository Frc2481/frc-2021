/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"



#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <frc/DriverStation.h>
#include <frc2/command/WaitCommand.h>

#include "Constants.h"
//Drive
#include "commands/Drive/DriveOpenLoopCommand.h"
#include "commands/Drive/DriveWithJoystickCommand.h"
#include "commands/Drive/RotateWithMotionMagic.h"
#include "commands/Drive/EngageBakeCommand.h"
//Intake
// #include "commands/Intake/SetBIntakeSpeed.h"
// #include "commands/Intake/SetAIntakeSpeed.h"
// #include "commands/Intake/IntakesDefaultCommand.h"
// #include "commands/Intake/ToggleIntakeCommand.h"
// #include "commands/Intake/DropBallsCommand.h"
// #include "commands/Intake/DropIntake.h"
// //Autos
#include "commands/Autos/TestSpeedsAuto.h"//T
// #include "commands/Autos/GalacticSearchPathABlue.h"
// #include "commands/Autos/GalacticSearchPathBBlue.h"
// #include "commands/Autos/GalacticSearchPathARed.h"
// #include "commands/Autos/GalacticSearchPathBRed.h"
// #include "networktables/NetworkTableInstance.h"

#include "commands/Autos/AutoNavPathA.h"//E
#include "commands/Autos/AutoNavPathB.h"//F
#include "commands/Autos/AutoNavPathC.h"//G


// #include "commands/Autos/AutoSelectorCommandPath.h"
using namespace DriveConstants;

RobotContainer::RobotContainer(): m_driverController(0),
                                  m_auxController(1),
                                  m_tDpadAux(&m_auxController, XBOX_DPAD_TOP),
                                  m_bDpadAux(&m_auxController, XBOX_DPAD_BOTTOM){
  ConfigureButtonBindings();

  m_drive.SetDefaultCommand(DriveWithJoystickCommand(&m_drive, &m_driverController));


  const double metersToInches = 39.3701;
  const double curveSmall = 17;//half of robot(12) + half of cone(2.5) + buffer(2.5)
  const double curveBigNavA = 32;//35// 27;//25//30
  const double radCurveA = 20;
  const double slipNavAA = -27.5;//-10;
  const double slipNavAB = -47.5;//-20;
  const double slipNavAXB = 15;//10
  const double slipNavAC = -50;//-10;
  const double angle = 180;
  //Auto Nav pathABlue A waypoints
  std::vector<SwerveDrivePathGenerator::waypoint_t> navPathA;//RobotParameters::k_maxSpeed*metersToInches           
  navPathA.push_back(SwerveDrivePathGenerator::waypoint_t {60  - RobotParameters::k_robotWidth/2, 80, angle, 0, 0});//start navPathB
  navPathA.push_back(SwerveDrivePathGenerator::waypoint_t {150 + slipNavAA + curveBigNavA, 60 + curveBigNavA, angle, RobotParameters::k_maxSpeed*metersToInches, radCurveA});//cone 1, point 1
  navPathA.push_back(SwerveDrivePathGenerator::waypoint_t {150 + slipNavAA + curveBigNavA, 60 - curveBigNavA, angle, RobotParameters::k_maxSpeed*metersToInches, radCurveA});//cone 1, point 2
  navPathA.push_back(SwerveDrivePathGenerator::waypoint_t {150 + slipNavAA - curveBigNavA, 60 - curveBigNavA, angle, RobotParameters::k_maxSpeed*metersToInches, radCurveA});//cone 1, point 3
  navPathA.push_back(SwerveDrivePathGenerator::waypoint_t {150 + slipNavAA - curveBigNavA, 60 + curveBigNavA, angle, RobotParameters::k_maxSpeed*metersToInches, radCurveA});//cone 1, point 4
  
  navPathA.push_back(SwerveDrivePathGenerator::waypoint_t {240 + slipNavAB + curveBigNavA, 120 + slipNavAXB - curveBigNavA, angle, RobotParameters::k_maxSpeed*metersToInches, radCurveA});//cone 2, point 1
  navPathA.push_back(SwerveDrivePathGenerator::waypoint_t {240 + slipNavAB + curveBigNavA, 120 + slipNavAXB + curveBigNavA, angle, RobotParameters::k_maxSpeed*metersToInches, radCurveA});//cone 2, point 2
  navPathA.push_back(SwerveDrivePathGenerator::waypoint_t {240 + slipNavAB - curveBigNavA, 120 + slipNavAXB + curveBigNavA, angle, RobotParameters::k_maxSpeed*metersToInches, radCurveA});//cone 2, point 3
  navPathA.push_back(SwerveDrivePathGenerator::waypoint_t {240 + slipNavAB - curveBigNavA, 120 + slipNavAXB - curveBigNavA, angle, RobotParameters::k_maxSpeed*metersToInches, radCurveA});//cone 2, point 4

  navPathA.push_back(SwerveDrivePathGenerator::waypoint_t {300 + slipNavAC - curveBigNavA, 60 + slipNavAXB - curveBigNavA, angle, RobotParameters::k_maxSpeed*metersToInches, radCurveA});//cone 3, point 1
  navPathA.push_back(SwerveDrivePathGenerator::waypoint_t {300 + slipNavAC + curveBigNavA, 60 + slipNavAXB - curveBigNavA, angle, RobotParameters::k_maxSpeed*metersToInches, radCurveA});//cone 3, point 2
  navPathA.push_back(SwerveDrivePathGenerator::waypoint_t {300 + slipNavAC + curveBigNavA, 60 + slipNavAXB + curveBigNavA, angle, RobotParameters::k_maxSpeed*metersToInches, radCurveA});//cone 3, point 3
  navPathA.push_back(SwerveDrivePathGenerator::waypoint_t {300 + slipNavAC - curveBigNavA, 60 + slipNavAXB + curveBigNavA, angle, RobotParameters::k_maxSpeed*metersToInches, radCurveA});//cone 3, point 4
  navPathA.push_back(SwerveDrivePathGenerator::waypoint_t {20 + slipNavAC, 90 + 25 + 20, angle, RobotParameters::k_maxSpeed*metersToInches, 0});//head to end
  navPathA.push_back(SwerveDrivePathGenerator::waypoint_t {20 + slipNavAC + RobotParameters::k_robotWidth/2, 90 + 25+20, angle, 0, 0});// x should be zero
  m_followerNavPathA.generatePath(navPathA,"navPathA");

  // const double curveBigNavB = 25;//22.5  25  32
  // const double radCurveB = 24;//12
  // const double curveSmallB = 20;
  // //Auto Nav Path B waypoints
  // const double fudge = -20;//20
  // const double x = -20;
  // const double y = -5;
  // const double circle = 5;//5;
  // std::vector<SwerveDrivePathGenerator::waypoint_t> navPathB;
  // navPathB.push_back(SwerveDrivePathGenerator::waypoint_t {30 + RobotParameters::k_robotWidth/2, 40, angle, 0, 0});//start
  // navPathB.push_back(SwerveDrivePathGenerator::waypoint_t {60 + curveSmallB, 60 - curveSmallB, angle, RobotParameters::k_maxSpeed*metersToInches, radCurveB});//over cone 1
  // navPathB.push_back(SwerveDrivePathGenerator::waypoint_t {120 - curveBigNavB, 60 + 32, angle, RobotParameters::k_maxSpeed*metersToInches, radCurveB});//over cone 1
  // // navPathB.push_back(SwerveDrivePathGenerator::waypoint_t {110, 90, angle, RobotParameters::k_maxSpeed*metersToInches, 0});//over cone 5

  // navPathB.push_back(SwerveDrivePathGenerator::waypoint_t {240 + fudge + curveSmallB, 60 + curveSmallB + 5, angle, RobotParameters::k_maxSpeed*metersToInches, 0});//over cone 5
  // navPathB.push_back(SwerveDrivePathGenerator::waypoint_t {300 + fudge - (curveBigNavB+circle), 60 - (curveBigNavB+circle), angle, RobotParameters::k_maxSpeed*metersToInches, radCurveB});//over cone 6  //small
  // navPathB.push_back(SwerveDrivePathGenerator::waypoint_t {300 + fudge + (curveBigNavB+circle), 60 - (curveBigNavB+circle), angle, RobotParameters::k_maxSpeed*metersToInches, radCurveB});//under cone 6
  // navPathB.push_back(SwerveDrivePathGenerator::waypoint_t {300 + fudge + (curveBigNavB+circle), 60 + (curveBigNavB+circle), angle, RobotParameters::k_maxSpeed*metersToInches, radCurveB});//under cone 6
  // navPathB.push_back(SwerveDrivePathGenerator::waypoint_t {300 + fudge - (curveBigNavB+circle), 60 + (curveBigNavB+circle), angle, RobotParameters::k_maxSpeed*metersToInches, radCurveB});//over cone 6  //small
  // navPathB.push_back(SwerveDrivePathGenerator::waypoint_t {240 + fudge + curveSmallB, 60 - curveSmallB + y, angle, RobotParameters::k_maxSpeed*metersToInches, 0});//under cone 5
  // navPathB.push_back(SwerveDrivePathGenerator::waypoint_t {85 + fudge + x + curveSmallB, 60 - curveSmallB + y, angle, RobotParameters::k_maxSpeed*metersToInches, 0});//under cone 1

  // navPathB.push_back(SwerveDrivePathGenerator::waypoint_t {70 + x + fudge, 100 + y, angle, RobotParameters::k_maxSpeed*metersToInches, radCurveB});//over cone 1
  // navPathB.push_back(SwerveDrivePathGenerator::waypoint_t {60 + x + fudge, 140 + y , angle, RobotParameters::k_maxSpeed*metersToInches, 0});//head to end
  // navPathB.push_back(SwerveDrivePathGenerator::waypoint_t {50 + x + fudge , 280 + y, angle, 0, 0});//head to end
  // // navPathB.push_back(SwerveDrivePathGenerator::waypoint_t {15, 150, angle, 0, 0});//decelerate x should be zero
  // // navPathB.push_back(SwerveDrivePathGenerator::waypoint_t {90 + fudge - 20/* 6*/, 60 - 0, angle, RobotParameters::k_maxSpeed*metersToInches, radCurveB});//over cone 1
  // // navPathB.push_back(SwerveDrivePathGenerator::waypoint_t {60 + curveBigNavB - 6, 60 + curveBigNavB, angle, RobotParameters::k_maxSpeed*metersToInches, radCurveB});//head to end
  // // navPathB.push_back(SwerveDrivePathGenerator::waypoint_t {60, 90 + 4 + 2 + RobotParameters::k_robotWidth/2, angle, RobotParameters::k_maxSpeed*metersToInches, radCurveB});//head to end
  // // navPathB.push_back(SwerveDrivePathGenerator::waypoint_t {30, 90 + 4 + 2 + RobotParameters::k_robotWidth/2, angle, 0, 0});//decelerate x should be zero
  // m_followerNavPathB.generatePath(navPathB,"navPathB");


// const double curveBigNavC = 26.0;//25
// const double slipA = 10.0;//10//-25;//40
// const double slipX = 8.0;//10;
// const double slipB = 18.0;//18 //15
// const double tempSpeed = RobotParameters::k_maxSpeed*metersToInches*1.0;//
// const double fakeStopSpeed = 20.0;
// const double radCurveC = 24;
// const double tempCurve = 0;//15;//12;
// const double cone = 5;
//   //Auto Nav Path C waypoints
//   std::vector<SwerveDrivePathGenerator::waypoint_t> navPathCA;
//   navPathCA.push_back(SwerveDrivePathGenerator::waypoint_t {32 + RobotParameters::k_robotWidth/2, 100, angle, 0, 0});//start
//   navPathCA.push_back(SwerveDrivePathGenerator::waypoint_t {32 + RobotParameters::k_robotWidth/2 + curveBigNavC, 100, angle, tempSpeed, 0});//clear start zone
//   navPathCA.push_back(SwerveDrivePathGenerator::waypoint_t {90, 150 - RobotParameters::k_robotWidth/2 - 2.5, angle, fakeStopSpeed, 0});//bounce point 1
//   m_followerNavPathC[0].generatePath(navPathCA, "pathBRed");
//   std::vector<SwerveDrivePathGenerator::waypoint_t> navPathCB;
//   navPathCB.push_back(SwerveDrivePathGenerator::waypoint_t {90, 150 - RobotParameters::k_robotWidth/2 - 2.5, angle, fakeStopSpeed, 0});//bounce point 1
//   navPathCB.push_back(SwerveDrivePathGenerator::waypoint_t {150 - curveBigNavC - tempCurve + 1,slipA +  60 - curveBigNavC - tempCurve+3, angle, tempSpeed, radCurveC});//curve around d5
//   navPathCB.push_back(SwerveDrivePathGenerator::waypoint_t {150 + curveBigNavC, slipA +  60 - curveBigNavC, angle, tempSpeed, radCurveC});//curve around d5
//   navPathCB.push_back(SwerveDrivePathGenerator::waypoint_t {180, slipA + 10 + 150 - RobotParameters::k_robotWidth/2 - cone , angle, fakeStopSpeed, 0});//bounce point 2
//   m_followerNavPathC[1].generatePath(navPathCB, "pathBRed");
//   std::vector<SwerveDrivePathGenerator::waypoint_t> navPathCC;
//   navPathCC.push_back(SwerveDrivePathGenerator::waypoint_t {180, 150 - RobotParameters::k_robotWidth/2- cone*2, angle, fakeStopSpeed, 0});//bounce point 2
//   navPathCC.push_back(SwerveDrivePathGenerator::waypoint_t {210 - curveBigNavC - tempCurve,slipB +   60 - curveBigNavC - 10, angle, tempSpeed, radCurveC});//curve around d7
//   navPathCC.push_back(SwerveDrivePathGenerator::waypoint_t {240 + slipX + curveBigNavC, 60 - curveBigNavC, angle, tempSpeed, radCurveC});//curve around d8
//   navPathCC.push_back(SwerveDrivePathGenerator::waypoint_t {270 + slipX, slipB + 10 + 150 + 1 - RobotParameters::k_robotWidth- cone, angle, fakeStopSpeed, 0});//bounce point 3
//   m_followerNavPathC[2].generatePath(navPathCC, "pathBRed");
//   std::vector<SwerveDrivePathGenerator::waypoint_t> navPathCD;
//   navPathCD.push_back(SwerveDrivePathGenerator::waypoint_t {270, 150 - RobotParameters::k_robotWidth- cone*2.5, angle, fakeStopSpeed, 0});//bounce point 3
//   navPathCD.push_back(SwerveDrivePathGenerator::waypoint_t {300 - curveBigNavC, slipB - 20 +  120 - curveBigNavC- RobotParameters::k_robotWidth/2, angle, tempSpeed, 0});//head to end
//   navPathCD.push_back(SwerveDrivePathGenerator::waypoint_t {300 + slipX, slipB - 45 +  100 - RobotParameters::k_robotWidth/2, angle, tempSpeed, 0});//curve into end
//   navPathCD.push_back(SwerveDrivePathGenerator::waypoint_t {360 + slipX, slipB - 45 +  100 - RobotParameters::k_robotWidth/2, angle, 0, 0});//decelerate
//   m_followerNavPathC[3].generatePath(navPathCD, "pathBRed");
//   std::vector<SwerveDrivePathGenerator::waypoint_t> navPathCE;
//   navPathCE.push_back(SwerveDrivePathGenerator::waypoint_t {360, 100, angle, 0, 0});//bounce point 3
//   navPathCE.push_back(SwerveDrivePathGenerator::waypoint_t {200, 100, angle, 30, 0});//bounce point 3
//   navPathCE.push_back(SwerveDrivePathGenerator::waypoint_t {30 - RobotParameters::k_robotWidth/2, 100, angle, 0, 0});//decelerate
//   m_followerNavPathC[4].generatePath(navPathCE, "pathBRed");


  //Galactic Search Path A Red waypoints
  // const int selectOffset = 4.5;
  // std::vector<SwerveDrivePathGenerator::waypoint_t> pathARed;
  // // pathARed.push_back(SwerveDrivePathGenerator::waypoint_t {43, 90 + selectOffset, 0, 0, 0});//start pathARed
  // // pathARed.push_back(SwerveDrivePathGenerator::waypoint_t {45, 90, 10, RobotParameters::k_maxSpeed*metersToInches, 0});//pick up ball 1 
  // // pathARed.push_back(SwerveDrivePathGenerator::waypoint_t {90, 90, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//pick up ball 1 
  // // pathARed.push_back(SwerveDrivePathGenerator::waypoint_t {150, 60, -26.5, RobotParameters::k_maxSpeed*metersToInches, 0});//pick up ball 2
  // // pathARed.push_back(SwerveDrivePathGenerator::waypoint_t {177/*180*/, 150, -26.5, RobotParameters::k_maxSpeed*metersToInches, 0});//pick up ball 3
  // // pathARed.push_back(SwerveDrivePathGenerator::waypoint_t {330, 150, -26.5, RobotParameters::k_maxSpeed*metersToInches, 0});//head to end
  // // pathARed.push_back(SwerveDrivePathGenerator::waypoint_t {360, 150, -26.5, 0, 0});//final 30in after endzone

  // pathARed.push_back(SwerveDrivePathGenerator::waypoint_t {43, 90 + selectOffset, 0, 0, 0});
  // pathARed.push_back(SwerveDrivePathGenerator::waypoint_t {43, 75, 10, 90, 0});
  // pathARed.push_back(SwerveDrivePathGenerator::waypoint_t {43, 60 + selectOffset, 0, 0, 0});


  // m_followerPathARed.generatePath(pathARed,"pathARed");

  // //Galactic Search Path B Red waypoints
  // std::vector<SwerveDrivePathGenerator::waypoint_t> pathBRed;
  // // pathBRed.push_back(SwerveDrivePathGenerator::waypoint_t {43.5, 120 - selectOffset, 0, 0, 0});//start pathBRed
  // // pathBRed.push_back(SwerveDrivePathGenerator::waypoint_t {90, 120, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//pick up ball 1
  // // pathBRed.push_back(SwerveDrivePathGenerator::waypoint_t {150, 60, -45, RobotParameters::k_maxSpeed*metersToInches, 0});//pick up ball 2
  // // pathBRed.push_back(SwerveDrivePathGenerator::waypoint_t {210, 120, -45, RobotParameters::k_maxSpeed*metersToInches, 0});//pick up ball 3
  // // pathBRed.push_back(SwerveDrivePathGenerator::waypoint_t {330, 120, -45, RobotParameters::k_maxSpeed*metersToInches, 0});//head to end
  // // pathBRed.push_back(SwerveDrivePathGenerator::waypoint_t {360, 120, -45, 0, 0});//final 30in after endzone

  // pathBRed.push_back(SwerveDrivePathGenerator::waypoint_t {43.5, 60 - selectOffset, 0, 0, 0});//start pathBRed
  // pathBRed.push_back(SwerveDrivePathGenerator::waypoint_t {43.5, 75, 0, 90, 0});//pick up ball 1
  // pathBRed.push_back(SwerveDrivePathGenerator::waypoint_t {43.5, 90 - selectOffset, 0, 0, 0});//final 30in after endzone


  // m_followerPathBRed.generatePath(pathBRed, "pathBRed");

  // //Galactic Search Path A Blue waypoints //selectOffset
  // const double slip = -10;
  // std::vector<SwerveDrivePathGenerator::waypoint_t> pathABlue;
  // pathABlue.push_back(SwerveDrivePathGenerator::waypoint_t {30, 30, 0, 0, 0});//start pathABlue
  // pathABlue.push_back(SwerveDrivePathGenerator::waypoint_t {35, 30, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//Drop Intakes
  // pathABlue.push_back(SwerveDrivePathGenerator::waypoint_t {55, 30, 15, RobotParameters::k_maxSpeed*metersToInches, 0});//Drop Intakes
  // pathABlue.push_back(SwerveDrivePathGenerator::waypoint_t {65, 30, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//Drop Intakes
  // pathABlue.push_back(SwerveDrivePathGenerator::waypoint_t {180, 30, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//pick up ball 1
  // pathABlue.push_back(SwerveDrivePathGenerator::waypoint_t {210, 120, -26.5, RobotParameters::k_maxSpeed*metersToInches, 0});//pick up ball 2
  // pathABlue.push_back(SwerveDrivePathGenerator::waypoint_t {270, 90 + slip, -26.5, RobotParameters::k_maxSpeed*metersToInches, 0});//pick up ball 3
  // pathABlue.push_back(SwerveDrivePathGenerator::waypoint_t {330, 90 + slip, -26.5, RobotParameters::k_maxSpeed*metersToInches, 0});//head to end
  // pathABlue.push_back(SwerveDrivePathGenerator::waypoint_t {360, 90 + slip, -26.5, 0, 0});//final 30in after endzone
  // m_followerPathABlue.generatePath(pathABlue,"pathABlue");

  // //Galactic Search Path B Blue waypoints
  // const double yaw = 45;
  // const double slipB = 0;
  // std::vector<SwerveDrivePathGenerator::waypoint_t> pathBBlue;
  // pathBBlue.push_back(SwerveDrivePathGenerator::waypoint_t {30, 60 - 5, 0, 0, 0});//start pathBBlue
  // pathBBlue.push_back(SwerveDrivePathGenerator::waypoint_t {35, 60 - 5, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//Drop Intake
  // pathBBlue.push_back(SwerveDrivePathGenerator::waypoint_t {45, 60 - 5, 15, RobotParameters::k_maxSpeed*metersToInches, 0});//Drop Intake
  // pathBBlue.push_back(SwerveDrivePathGenerator::waypoint_t {55, 60 - 5, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//Drop Intake
  // pathBBlue.push_back(SwerveDrivePathGenerator::waypoint_t {180, 60 - 5, 0, RobotParameters::k_maxSpeed*metersToInches, 0});//pick up ball 1
  // pathBBlue.push_back(SwerveDrivePathGenerator::waypoint_t {240, 120, -yaw + 10, RobotParameters::k_maxSpeed*metersToInches, 0});//pick up ball 2 -45
  // pathBBlue.push_back(SwerveDrivePathGenerator::waypoint_t {300, 60, -yaw, RobotParameters::k_maxSpeed*metersToInches, 0});//pick up ball 3
  // pathBBlue.push_back(SwerveDrivePathGenerator::waypoint_t {330, 60, -yaw, RobotParameters::k_maxSpeed*metersToInches, 0});//head to end
  // pathBBlue.push_back(SwerveDrivePathGenerator::waypoint_t {360, 60, -yaw, 0, 0});//final 30in after endzone
  // m_followerPathBBlue.generatePath(pathBBlue,"pathBBlue");
  
  

}

class InstantDisabledCommand : public frc2::InstantCommand {
public:

  InstantDisabledCommand(std::function<void()> toRun,
                 std::initializer_list<frc2::Subsystem*> requirements = {}) : frc2::InstantCommand(toRun, requirements) {} 

  virtual bool RunsWhenDisabled() const override {
    return true;
  }
};


void RobotContainer::ConfigureButtonBindings(){
  frc::SmartDashboard::PutData("Reset Drive Encoders", new InstantDisabledCommand([this](){
    m_drive.resetDriveEncoders();
  }));
  frc::SmartDashboard::PutData("Set Coast", new InstantDisabledCommand([this](){
    m_drive.setCoast();
  }));
  frc::SmartDashboard::PutData("Set Brake", new InstantDisabledCommand([this](){
    m_drive.setBrake();
  }));
  frc::SmartDashboard::PutData("Zero Steer Encoders", new InstantDisabledCommand([this](){
    m_drive.ResetEncoders();
  }));

  frc::SmartDashboard::PutData("Reset Odometry", new InstantDisabledCommand([this](){
    m_drive.ResetOdometry(frc::Pose2d());
  }));
  //driver
    m_startDriver.WhenPressed(new frc2::InstantCommand([this]{
                              m_drive.ResetOdometry(frc::Pose2d(
                                                                m_drive.GetPose().Translation().X(), 
                                                                m_drive.GetPose().Translation().Y(),
                                                                frc::Rotation2d(units::degree_t(0))));
                              },{&m_drive}));
                    
    m_lBumperDriver.WhenPressed(new frc2::InstantCommand([this]{m_drive.toggleFieldCentricForJoystick();},{&m_drive}));
    // m_yButtonDriver.WhenPressed(new IntakesDefaultCommand(&m_intake));//ToggleIntakeCommand(&m_intake));
    // m_bButtonDriver.WhileHeld(new DropBallsCommand(&m_intake));
    // m_aButtonDriver.WhenPressed(new DropIntake(&m_intake));
    // m_rTriggerDriver.WhileHeld(new IntakesDefaultCommand(&m_intake));
    //operator
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // return new EngageBakeCommand(&m_drive);
  // if((bool)nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv",0)){
  //     double targetDistance = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty",0);
  //     double targetY = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0);
  //     frc::SmartDashboard::PutNumber("target Distance", targetDistance);
  //     frc::SmartDashboard::PutNumber("target Y", targetY);
  //     if(targetDistance < 0){//RED
  //       if(targetY > 0){
  //         frc::SmartDashboard::PutString("PATH", "A Red");
  //         // printf("A Red");
  //         return new GalacticSearchPathARed(&m_drive, &m_intake, &m_followerPathARed);
  //       }else{
  //         frc::SmartDashboard::PutString("PATH", "B Red");
  //         // printf("B Red");
  //         return new GalacticSearchPathBRed(&m_drive, &m_intake, &m_followerPathBRed);
  //       }
  //     }else if(targetDistance > 0){//Blue
  //       if(targetY > 0){
  //         frc::SmartDashboard::PutString("PATH", "A Blue");
  //         // printf("A Blue");
  //         return new GalacticSearchPathABlue(&m_drive, &m_intake, &m_followerPathABlue);
  //       }else{
  //         frc::SmartDashboard::PutString("PATH", "B Blue");
  //         // printf("B Blue");
  //         return new GalacticSearchPathBBlue(&m_drive, &m_intake, &m_followerPathBBlue);
  //       }
  //     }
  //   }
  //     return new GalacticSearchPathABlue(&m_drive, &m_intake, &m_followerPathABlue);
    
  // return new AutoSelectorCommandPath(&m_drive, &m_intake, &m_followerPathABlue, &m_followerPathBBlue, &m_followerPathARed, &m_followerPathBRed);
  // std::string gameData;
  // gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
  // if(gameData.length() > 0){
    
  //   switch(gameData[0]){
  //     // case 'A':
  //     //   return new GalacticSearchPathARed(&m_drive, &m_intake, &m_followerPathARed);
  //     // case 'B':
  //     //   return new GalacticSearchPathBRed(&m_drive, &m_intake, &m_followerPathBRed);
  //     // case 'C':
  //     //   return new GalacticSearchPathABlue(&m_drive, &m_intake, &m_followerPathABlue);
  //     // case 'D':
  //     //   return new GalacticSearchPathBBlue(&m_drive, &m_intake, &m_followerPathBBlue);
  //     case 'A':
        return new AutoNavPathA(&m_drive, &m_followerNavPathA);
  //     case 'B':
        // return new AutoNavPathB(&m_drive, &m_followerNavPathB);
  //     case 'C':
        //  return new AutoNavPathC(&m_drive, m_followerNavPathC);
  //     case 'T':
        // return new TestSpeedsAuto(&m_drive, &m_intake);
  //     default:
  //       return new GalacticSearchPathARed(&m_drive, &m_intake, &m_followerPathARed);
  //   }
  // }else{
    // return new AutoNavPathC(&m_drive, m_followerNavPathC);
  // }
}

frc2::InstantCommand* RobotContainer::GetBrakeCommand(){
  return new frc2::InstantCommand([this]{m_drive.setBrake(); },{&m_drive});
}
frc2::InstantCommand* RobotContainer::GetCoastCommand(){
  return new frc2::InstantCommand([this]{m_drive.setCoast();},{&m_drive});
}