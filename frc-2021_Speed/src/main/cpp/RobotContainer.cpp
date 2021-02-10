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
#include <units/units.h>
#include "Constants.h"
#include <frc2/command/WaitCommand.h>


// #include "commands/ReadColorSensorCommand.h"
// #include "commands/RotatePanelCommand.h"
// #include "commands/RotatePanelToColor.h"


#include "commands/Drive/DriveOpenLoopCommand.h"
#include "commands/Drive/RotateToAngleCommand.h"
#include "commands/Drive/DriveWithJoystickCommand.h"
#include "commands/Drive/RotateWithMotionMagic.h"

// #include "commands/LimeLight/LimeLightRotateToTargetCommand.h"
// #include "commands/LimeLight/LimeLightSwerveDriveRotateToTargetCommand.h"
// #include "commands/LimeLight/LimelightUpdatePose.h"

#include "commands/Intake/CheckBeamBreak.h"
#include "commands/Intake/SetLeftIntakeSpeed.h"
#include "commands/Intake/SetRightIntakeSpeed.h"

#include "commands/Autos/AutoLeftCommandGroup.h"

using namespace DriveConstants;

RobotContainer::RobotContainer(): m_driverController(0),
                                  m_auxController(1),
                                  m_tDpadAux(&m_auxController, XBOX_DPAD_TOP),
                                  m_bDpadAux(&m_auxController, XBOX_DPAD_BOTTOM){
  ConfigureButtonBindings();
frc::SmartDashboard::PutData("motion magic to angle", new RotateWithMotionMagic(&m_drive, 20,1,false));
  // frc::SmartDashboard::PutNumber("drive P", 0.1);//5
  // frc::SmartDashboard::PutNumber("drive I", 0);
  // frc::SmartDashboard::PutNumber("drive D", 0);//10
  // frc::SmartDashboard::PutNumber("drive F", 1023/(RobotParameters::k_maxSpeed/RobotParameters::k_driveMotorEncoderTicksToMPS));//10

  
  // frc::SmartDashboard::PutData("updatDrivePID", new frc2::InstantCommand([this]{
  //   m_drive.tuneDrivePID(frc::SmartDashboard::GetNumber("drive P", .1),
  //               frc::SmartDashboard::GetNumber("drive I", 0),
  //               frc::SmartDashboard::GetNumber("drive D", 0),
  //               frc::SmartDashboard::GetNumber("drive F", 1023/(RobotParameters::k_maxSpeed/RobotParameters::k_driveMotorEncoderTicksToMPS)));
  // },{&m_drive}));

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
    //operator

}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return new AutoLeftCommandGroup(&m_follower, &m_drive);
}

frc2::InstantCommand* RobotContainer::GetBrakeCommand(){
  return new frc2::InstantCommand([this]{m_drive.setBrake(); },{&m_drive});//TurnLimeLightOff();
}
  frc2::InstantCommand* RobotContainer::GetCoastCommand(){
    return new frc2::InstantCommand([this]{m_drive.setCoast();},{&m_drive});
  }