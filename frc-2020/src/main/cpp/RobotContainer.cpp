/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"


#include "commands/shooter/StartShooterCommand.h"
#include "commands/shooter/StopShooterCommand.h"
#include "commands/Shooter/DecShooterSpeedCommand.h"
#include "commands/Shooter/IncShooterSpeedCommand.h"
#include "commands/Shooter/ShootBallCommand.h"
#include "commands/shooter/ManualShootBallCommand.h"
#include "commands/shooter/ShortShotCommand.h"
#include "commands/shooter/FarShotCommand.h"
#include "commands/shooter/SetFarShotSpeedCommand.h"
#include "commands/shooter/SetCloseShotSpeedCommand.h"
#include "commands/shooter/AutoSetShootSpeedCommand.h"
#include "commands/shooter/ShootBallFarCommand.h"
#include "commands/shooter/SetShooterSpeedCommand.h"
#include "commands/shooter/ToggleDinkerCommand.h"


#include "commands/pathCommands/AutoSwerveFollowPathCommand.h"
#include "commands/pathCommands/WaitForPathToFinishCommand.h"
#include "commands/pathCommands/PathFollowerCommand.h"
#include "commands/pathCommands/StartPathFollowing.h"
#include "commands/pathCommands/StopPathFollowing.h"
#include "commands/pathCommands/WaitForPosCommand.h"


#include "commands/Climber/ClimbMidCommand.h"
#include "commands/Climber/ClimberPullUpCommand.h"
#include "commands/Climber/ClimbHighCommand.h"
#include "commands/Climber/ClimbWithJoyStickCommand.h"
#include "commands/Climber/ClimberToPos.h"


#include "commands/Autos/AutoLeftCommandGroup.h"
// #include "commands/Autos/AutoRightCommandGroup.h"
// #include "commands/Autos/AutoTestCommandGroup.h"


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

#include "commands/LimeLight/LimeLightRotateToTargetCommand.h"
#include "commands/LimeLight/LimeLightSwerveDriveRotateToTargetCommand.h"
#include "commands/LimeLight/LimelightUpdatePose.h"

// #include "commands/Intake/SetFeederSpeedCommand.h"
// #include "commands/Intake/SetRollerSpeedCommand.h"
// #include "commands/Intake/SetIndexerSpeedCommand.h"
#include "commands/Intake/ZeroBallCountCommand.h"
#include "commands/Intake/ManualIntakeBallsCommand.h"
#include "commands/Intake/RetractIntakeCommand.h"
#include "commands/Intake/ExtendIntakeCommand.h"
#include "commands/Intake/UnjamCommand.h"
#include "commands/Intake/ResetFeederCommand.h"
#include "commands/Intake/FeederDefaultCommand.h"
#include "commands/Intake/IndexerDefaultCommand.h"



using namespace DriveConstants;

RobotContainer::RobotContainer(): m_driverController(0),
                                  m_auxController(1),
                                  m_tDpadAux(&m_auxController, XBOX_DPAD_TOP),
                                  m_bDpadAux(&m_auxController, XBOX_DPAD_BOTTOM){
  ConfigureButtonBindings();
  // frc::SmartDashboard::PutData("swerve drive sub", &m_drive);
  frc::SmartDashboard::PutData("rotate to angle", new RotateWithTrajectoryCommand(&m_drive,frc::SmartDashboard::GetNumber("angle to rotate", 0),.55));
  
  frc::SmartDashboard::PutNumber("ball distance", FeederConstants::kIntakeBallDistance);
  frc::SmartDashboard::GetNumber("ball range", .1);
  frc::SmartDashboard::PutNumber("shooter target Zone", 35);
  frc::SmartDashboard::PutNumber("shooter P", RobotParameters::k_shooterP);//5
  frc::SmartDashboard::PutNumber("shooter I", RobotParameters::k_shooterI);
  frc::SmartDashboard::PutNumber("shooter D", RobotParameters::k_shooterD);//10
  frc::SmartDashboard::PutNumber("shooter F", RobotParameters::k_shooterF);
  frc::SmartDashboard::PutNumber("IZone", 25);
  frc::SmartDashboard::PutNumber("shooter Set Point", 0 );
  
  frc::SmartDashboard::PutNumber("limeLight P", RobotParameters::k_limeLightP);
  frc::SmartDashboard::PutNumber("limeLight I", RobotParameters::k_limeLightI);
  frc::SmartDashboard::PutNumber("limeLight D", RobotParameters::k_limeLightD);
  frc::SmartDashboard::PutNumber("limeLight Izone", RobotParameters::k_limeLightIZone);

  frc::SmartDashboard::PutNumber("Lime light X", 0);
    frc::SmartDashboard::PutNumber("Lime light Y", 0);
    frc::SmartDashboard::PutNumber("Lime light yaw", 0);
  frc::SmartDashboard::PutData("limelight to angle", new LimeLightRotateToTargetCommand(&m_drive,.01));
  frc::SmartDashboard::PutNumber("motion magic target angle", 0);
  frc::SmartDashboard::PutData("motion magic to angle", new RotateWithMotionMagic(&m_drive, 20,1,true));

  frc::SmartDashboard::PutData("updateShooterPID", new frc2::InstantCommand([this]{
    m_shooter.updatePID(frc::SmartDashboard::GetNumber("shooter P", RobotParameters::k_shooterP),
                        frc::SmartDashboard::GetNumber("shooter I", RobotParameters::k_shooterI),
                        frc::SmartDashboard::GetNumber("shooter D", RobotParameters::k_shooterD),
                        frc::SmartDashboard::GetNumber("shooter F", RobotParameters::k_shooterF),
                        frc::SmartDashboard::GetNumber("IZone", 25));
  },{&m_shooter}));


  frc::SmartDashboard::PutNumber("feeder P", RobotParameters::k_feederP);//5
  frc::SmartDashboard::PutNumber("feeder I", RobotParameters::k_feederI);
  frc::SmartDashboard::PutNumber("feeder D", RobotParameters::k_feederD);//10
  frc::SmartDashboard::PutNumber("feeder F", RobotParameters::k_feederF);
  frc::SmartDashboard::PutNumber("IZone", 25);

  frc::SmartDashboard::PutData("updateFeederPID", new frc2::InstantCommand([this]{
    m_feeder.tunePID(frc::SmartDashboard::GetNumber("feeder P", RobotParameters::k_feederP),
                        frc::SmartDashboard::GetNumber("feeder I", RobotParameters::k_feederI),
                        frc::SmartDashboard::GetNumber("feeder D", RobotParameters::k_feederD),
                        frc::SmartDashboard::GetNumber("feeder F", RobotParameters::k_feederF),
                        frc::SmartDashboard::GetNumber("IZone", 25));
  },{&m_feeder}));

  frc::SmartDashboard::PutNumber("feeder setpoint", FeederConstants::kIntakeBallDistance);
  frc::SmartDashboard::PutData("feed to setpoint", new frc2::InstantCommand([this]{
    m_feeder.setFeederPos(frc::SmartDashboard::GetNumber("feeder setpoint", FeederConstants::kIntakeBallDistance));
  },{&m_feeder}));
  frc::SmartDashboard::PutNumber("shooting feeder speed", FeederConstants::kDefaultFeederSpeed);
  frc::SmartDashboard::PutNumber("feeder last ball distance", FeederConstants::kLastBallDistance);
  
  frc::SmartDashboard::PutNumber("climb P", RobotParameters::k_shooterP);//5
  frc::SmartDashboard::PutNumber("climb I", RobotParameters::k_shooterI);
  frc::SmartDashboard::PutNumber("climb D", RobotParameters::k_shooterD);//10
  frc::SmartDashboard::PutNumber("climb F", RobotParameters::k_climbF);//10
  frc::SmartDashboard::PutNumber("climb izone", RobotParameters::k_shooterF);

  
  frc::SmartDashboard::PutData("updatClimbPID", new frc2::InstantCommand([this]{
    m_climber.pid(frc::SmartDashboard::GetNumber("climb P", RobotParameters::k_climbP),
                        frc::SmartDashboard::GetNumber("climb I", RobotParameters::k_climbI),
                        frc::SmartDashboard::GetNumber("climb D", RobotParameters::k_climbD),
                        frc::SmartDashboard::GetNumber("climb F", RobotParameters::k_climbF),
                        frc::SmartDashboard::GetNumber("IZone", 25));
  },{&m_climber}));
  
  frc::SmartDashboard::PutData("zero climb", new frc2::InstantCommand([this]{
    m_climber.zeroPos();
  },{&m_climber}));
   
   frc::SmartDashboard::PutNumber("climberSetPos", 0);
   frc::SmartDashboard::PutData("climb to pos", new frc2::InstantCommand([this]{
    m_climber.goToPos(frc::SmartDashboard::GetNumber("climberSetPos", 0));
  },{&m_climber}));


  frc::SmartDashboard::PutNumber("drive P", 0.1);//5
  frc::SmartDashboard::PutNumber("drive I", 0);
  frc::SmartDashboard::PutNumber("drive D", 0);//10
  frc::SmartDashboard::PutNumber("drive F", 1023/(RobotParameters::k_maxSpeed/RobotParameters::k_driveMotorEncoderTicksToMPS));//10

  
  frc::SmartDashboard::PutData("updatDrivePID", new frc2::InstantCommand([this]{
    m_drive.tuneDrivePID(frc::SmartDashboard::GetNumber("drive P", .1),
                frc::SmartDashboard::GetNumber("drive I", 0),
                frc::SmartDashboard::GetNumber("drive D", 0),
                frc::SmartDashboard::GetNumber("drive F", 1023/(RobotParameters::k_maxSpeed/RobotParameters::k_driveMotorEncoderTicksToMPS)));
  },{&m_drive}));

  m_drive.SetDefaultCommand(DriveWithJoystickCommand(&m_drive, &m_driverController)); 
  m_feeder.SetDefaultCommand(FeederDefaultCommand(&m_feeder));
  m_indexer.SetDefaultCommand(IndexerDefaultCommand(&m_indexer, &m_feeder));
  m_climber.SetDefaultCommand(ClimbWithJoyStickCommand(&m_climber, &m_auxController));

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
  frc::SmartDashboard::PutData("climb command", new  ClimberToPos(&m_climber, frc::SmartDashboard::GetNumber("climberSetPos", 0)));
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
  frc::SmartDashboard::PutData("Limelight Update Pose", new InstantDisabledCommand([this](){
    new LimelightUpdatePose(&m_drive);
  }));
  frc::SmartDashboard::PutData("zero Ball Count", new ZeroBallCountCommand(&m_feeder));
  //driver
    m_startDriver.WhenPressed(new frc2::InstantCommand([this]{
                              m_drive.ResetOdometry(frc::Pose2d(
                                                                m_drive.GetPose().Translation().X(), 
                                                                m_drive.GetPose().Translation().Y(),
                                                                frc::Rotation2d(units::degree_t(0))));
                              },{&m_drive}));
    m_backDriver.WhenPressed(new ResetFeederCommand(&m_feeder));
    m_aButtonDriver.WhenPressed(new LimelightUpdatePose(&m_drive));
    m_bButtonDriver.WhileHeld(new LimeLightSwerveDriveRotateToTargetCommand(&m_drive,&m_driverController, 1));
    m_yButtonDriver.WhileHeld(new LimeLightRotateToTargetCommand(&m_drive,.25));  //WhenPressed(new ClimberPullUpCommand(&m_climb));
    m_xButtonDriver.WhileHeld(new ManualIntakeBallsCommand(&m_intake, &m_indexer, &m_feeder, false));
    // m_rBumperDriver.WhenPressed(new 
    m_lBumperDriver.WhenPressed(new frc2::InstantCommand([this]{m_drive.toggleFieldCentricForJoystick();},{&m_drive}));
    m_rTriggerDriver.WhenPressed(new ExtendIntakeCommand(&m_intake));
    m_rTriggerDriver.WhenReleased(new RetractIntakeCommand(&m_intake));
    // m_lTriggerDriver.       
    
                    

    //operator
    m_startAux.WhenPressed(new ClimbHighCommand(&m_climber));
    m_startAux.WhenReleased(new ClimberPullUpCommand(&m_climber));
    m_backAux.WhileHeld(new UnjamCommand(&m_indexer, &m_feeder));
    m_aButtonAux.WhenPressed(new AutoSetShootSpeedCommand(&m_shooter));
    m_bButtonAux.WhenPressed(new StopShooterCommand(&m_shooter));
    m_yButtonAux.ToggleWhenPressed(new ToggleDinkerCommand(&m_shooter));
    m_xButtonAux.WhileHeld(new ManualShootBallCommand(&m_shooter, &m_feeder, &m_indexer));
    m_rBumperAux.WhenPressed(new SetFarShotSpeedCommand(&m_shooter));
    m_lBumperAux.WhenPressed(new SetCloseShotSpeedCommand(&m_shooter));
    m_lTriggerAux.WhileHeld(new ShootBallCommand(&m_shooter, &m_feeder));//
    m_rTriggerAux.WhileHeld(new ShootBallFarCommand(&m_shooter, &m_feeder));
    m_tDpadAux.WhenPressed(new IncShooterSpeedCommand(&m_shooter));
    m_bDpadAux.WhenPressed(new DecShooterSpeedCommand(&m_shooter));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  
  // return nullptr;
 return new AutoLeftCommandGroup(&m_follower, &m_drive, &m_shooter, &m_feeder, &m_intake);
}

frc2::InstantCommand* RobotContainer::GetBrakeCommand(){
  return new frc2::InstantCommand([this]{m_drive.setBrake(); TurnLimeLightOff();},{&m_drive});
}
  frc2::InstantCommand* RobotContainer::GetCoastCommand(){
    return new frc2::InstantCommand([this]{m_drive.setCoast();},{&m_drive});
  }
