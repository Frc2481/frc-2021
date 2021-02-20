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
// #include <units/
#include "Constants.h"
#include <frc2/command/WaitCommand.h>


#include "commands/Drive/DriveOpenLoopCommand.h"
#include "commands/Drive/RotateToAngleCommand.h"
#include "commands/Drive/DriveWithJoystickCommand.h"
#include "commands/Drive/RotateWithMotionMagic.h"

#include "commands/LimeLight/LimelightUpdatePose.h"

#include "commands/Intake/SetLeftIntakeSpeed.h"
#include "commands/Intake/SetRightIntakeSpeed.h"
#include "commands/Intake/IntakesDefaultCommand.h"
#include "commands/Intake/ToggleIntakeCommand.h"
#include "commands/Intake/DropBallsCommand.h"
#include "commands/Intake/DropIntake.h"

#include "commands/Autos/AutoLeftCommandGroup.h"
#include "commands/Autos/TestSpeedsAuto.h"//T
#include "commands/Autos/GalacticSearchPathARed.h"//A default
#include "commands/Autos/GalacticSearchPathBRed.h"//B
#include "commands/Autos/GalacticSearchPathABlue.h"//C
#include "commands/Autos/GalacticSearchPathBBlue.h"//D
#include "commands/Autos/AutoNavPathA.h"//E
#include "commands/Autos/AutoNavPathB.h"//F
#include "commands/Autos/AutoNavPathC.h"//G

#include <frc/DriverStation.h>

using namespace DriveConstants;

RobotContainer::RobotContainer(): m_driverController(0),
                                  m_auxController(1),
                                  m_tDpadAux(&m_auxController, XBOX_DPAD_TOP),
                                  m_bDpadAux(&m_auxController, XBOX_DPAD_BOTTOM){
  ConfigureButtonBindings();
  frc::SmartDashboard::PutData("motion magic to angle", new RotateWithMotionMagic(&m_drive, 20,1,false));

  m_drive.SetDefaultCommand(DriveWithJoystickCommand(&m_drive, &m_driverController)); 
  // m_intake.SetDefaultCommand(IntakesDefaultCommand(&m_intake));
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
  
  frc::SmartDashboard::PutData("Get Current of 9", new InstantDisabledCommand([this](){
    frc::SmartDashboard::PutNumber("Current 9", m_PDP.GetCurrent(9));
  }));
  frc::SmartDashboard::PutData("Zero Steer Encoders", new InstantDisabledCommand([this](){
    m_drive.ResetEncoders();
  }));

  frc::SmartDashboard::PutData("Reset Odometry", new InstantDisabledCommand([this](){
    m_drive.ResetOdometry(frc::Pose2d());
  }));
  frc::SmartDashboard::PutData("Reset Odometry -90", new InstantDisabledCommand([this](){
    m_drive.ResetOdometry(frc::Pose2d(m_drive.GetPose().Translation().X(), 
                                                                m_drive.GetPose().Translation().Y(),
                                                                frc::Rotation2d(units::degree_t(-90))));
  }));
  //driver
    m_startDriver.WhenPressed(new frc2::InstantCommand([this]{
                              m_drive.ResetOdometry(frc::Pose2d(
                                                                m_drive.GetPose().Translation().X(), 
                                                                m_drive.GetPose().Translation().Y(),
                                                                frc::Rotation2d(units::degree_t(0))));
                              },{&m_drive}));
                    
    m_lBumperDriver.WhenPressed(new frc2::InstantCommand([this]{m_drive.toggleFieldCentricForJoystick();},{&m_drive}));
    m_yButtonDriver.WhenPressed(new IntakesDefaultCommand(&m_intake, &m_PDP));//ToggleIntakeCommand(&m_intake));
    m_bButtonDriver.WhileHeld(new DropBallsCommand(&m_intake));
    m_aButtonDriver.WhenPressed(new DropIntake(&m_intake));
    m_rTriggerDriver.WhileHeld(new IntakesDefaultCommand(&m_intake, &m_PDP));
    //operator
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  std::string gameData;
  gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
  if(gameData.length() > 0){
    
    switch(gameData[0]){
      case 'A':
        return new GalacticSearchPathARed(&m_follower, &m_drive, &m_intake, &m_PDP);
      case 'B':
        return new GalacticSearchPathBRed(&m_follower, &m_drive, &m_intake, &m_PDP);
      case 'C':
        return new GalacticSearchPathABlue(&m_follower, &m_drive, &m_intake, &m_PDP);
      case 'D':
        return new GalacticSearchPathBBlue(&m_follower, &m_drive, &m_intake, &m_PDP);
      case 'E':
        return new AutoNavPathA(&m_follower, &m_drive);
      case 'F':
        return new AutoNavPathB(&m_follower, &m_drive);
      case 'G':
        return new AutoNavPathC(&m_follower, &m_drive);
      case 'T':
        return new TestSpeedsAuto(&m_follower, &m_drive);
      default:
        return new GalacticSearchPathARed(&m_follower, &m_drive, &m_intake, &m_PDP);
    }
  }else{
    return new GalacticSearchPathARed(&m_follower, &m_drive, &m_intake, &m_PDP);
  }
  
}

frc2::InstantCommand* RobotContainer::GetBrakeCommand(){
  return new frc2::InstantCommand([this]{m_drive.setBrake(); },{&m_drive});//TurnLimeLightOff();
}
  frc2::InstantCommand* RobotContainer::GetCoastCommand(){
    return new frc2::InstantCommand([this]{m_drive.setCoast();},{&m_drive});
  }