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
//Intake
#include "commands/Intake/SetBIntakeSpeed.h"
#include "commands/Intake/SetAIntakeSpeed.h"
#include "commands/Intake/IntakesDefaultCommand.h"
#include "commands/Intake/ToggleIntakeCommand.h"
#include "commands/Intake/DropBallsCommand.h"
#include "commands/Intake/DropIntake.h"
//Autos
#include "commands/Autos/TestSpeedsAuto.h"//T
#include "commands/Autos/GalacticSearchPathARed.h"//A default
#include "commands/Autos/GalacticSearchPathBRed.h"//B
#include "commands/Autos/GalacticSearchPathABlue.h"//C
#include "commands/Autos/GalacticSearchPathBBlue.h"//D
#include "commands/Autos/AutoNavPathA.h"//E
#include "commands/Autos/AutoNavPathB.h"//F
#include "commands/Autos/AutoNavPathC.h"//G

using namespace DriveConstants;

RobotContainer::RobotContainer(): m_driverController(0),
                                  m_auxController(1),
                                  m_tDpadAux(&m_auxController, XBOX_DPAD_TOP),
                                  m_bDpadAux(&m_auxController, XBOX_DPAD_BOTTOM){
  ConfigureButtonBindings();

  m_drive.SetDefaultCommand(DriveWithJoystickCommand(&m_drive, &m_driverController));
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
    m_yButtonDriver.WhenPressed(new IntakesDefaultCommand(&m_intake));//ToggleIntakeCommand(&m_intake));
    m_bButtonDriver.WhileHeld(new DropBallsCommand(&m_intake));
    m_aButtonDriver.WhenPressed(new DropIntake(&m_intake));
    m_rTriggerDriver.WhileHeld(new IntakesDefaultCommand(&m_intake));
    //operator
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  std::string gameData;
  gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
  if(gameData.length() > 0){
    
    switch(gameData[0]){
      case 'A':
        return new GalacticSearchPathARed(&m_drive, &m_intake);
      case 'B':
        return new GalacticSearchPathBRed(&m_drive, &m_intake);
      case 'C':
        return new GalacticSearchPathABlue(&m_drive, &m_intake);
      case 'D':
        return new GalacticSearchPathBBlue(&m_drive, &m_intake);
      case 'E':
        return new AutoNavPathA(&m_drive);
      case 'F':
        return new AutoNavPathB(&m_drive);
      case 'G':
        return new AutoNavPathC(&m_drive);
      case 'T':
        return new TestSpeedsAuto(&m_drive, &m_intake);
      default:
        return new GalacticSearchPathARed(&m_drive, &m_intake);
    }
  }else{
    return new GalacticSearchPathARed(&m_drive, &m_intake);
  }
}

frc2::InstantCommand* RobotContainer::GetBrakeCommand(){
  return new frc2::InstantCommand([this]{m_drive.setBrake(); },{&m_drive});
}
frc2::InstantCommand* RobotContainer::GetCoastCommand(){
  return new frc2::InstantCommand([this]{m_drive.setCoast();},{&m_drive});
}