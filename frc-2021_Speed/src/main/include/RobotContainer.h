/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/XboxController.h>

#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/trajectory/Trajectory.h>

#include<frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/button/Button.h>

#include "components/Joystick2481.h"

#include <frc2/command/button/POVButton.h>
#include "Utils/SwerveDrivePathGenerator.h"
#include "Utils/SwerveDrivePathFollower.h"

#include "components/XboxController2481.h"

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/DriveSubsystem.h"


class RobotContainer {
 public:
  RobotContainer();
  frc2::InstantCommand* GetBrakeCommand();
  frc2::InstantCommand* GetCoastCommand();
  frc2::Command* GetAutonomousCommand();
  
 private:
  Joystick2481 m_driverController;
  Joystick2481 m_auxController;

  DriveSubsystem m_drive;
  IntakeSubsystem m_intake;
  SwerveDrivePathFollower m_follower;

  
  frc2::Button m_startDriver{[&] { return m_driverController.GetRawButton(XBOX_START_BUTTON); }};//
  frc2::Button m_backDriver{[&] { return m_driverController.GetRawButton(XBOX_BACK_BUTTON); }};//

  frc2::Button m_aButtonDriver{[&] { return m_driverController.GetRawButton(XBOX_A_BUTTON); }};
  frc2::Button m_bButtonDriver{[&] { return m_driverController.GetRawButton(XBOX_B_BUTTON); }};
  frc2::Button m_yButtonDriver{[&] { return m_driverController.GetRawButton(XBOX_Y_BUTTON); }};
  frc2::Button m_xButtonDriver{[&] { return m_driverController.GetRawButton(XBOX_X_BUTTON); }};

  frc2::Button m_rBumperDriver{[&] { return m_driverController.GetRawButton(XBOX_RIGHT_BUMPER); }};//
  frc2::Button m_lBumperDriver{[&] { return m_driverController.GetRawButton(XBOX_LEFT_BUMPER); }};//
  frc2::Button m_rTriggerDriver{[&] { return m_driverController.GetAxis(XBOX_RIGHT_TRIGGER, .5); }};//
  frc2::Button m_lTriggerDriver{[&] { return m_driverController.GetAxis(XBOX_LEFT_TRIGGER, .5); }};//

  //operator
  frc2::Button m_startAux{[&] { return m_auxController.GetRawButton(XBOX_START_BUTTON); }};//
  frc2::Button m_backAux{[&] { return m_auxController.GetRawButton(XBOX_BACK_BUTTON); }};//

  frc2::Button m_aButtonAux{[&] { return m_auxController.GetRawButton(XBOX_A_BUTTON); }};//

  frc2::Button m_bButtonAux{[&] { return m_auxController.GetRawButton(XBOX_B_BUTTON); }};//
  frc2::Button m_yButtonAux{[&] { return m_auxController.GetRawButton(XBOX_Y_BUTTON); }};//
  frc2::Button m_xButtonAux{[&] { return m_auxController.GetRawButton(XBOX_X_BUTTON); }};//
  
  frc2::Button m_rBumperAux{[&] { return m_auxController.GetRawButton(XBOX_RIGHT_BUMPER); }};//
  frc2::Button m_lBumperAux{[&] { return m_auxController.GetRawButton(XBOX_LEFT_BUMPER); }};//
  frc2::Button m_rTriggerAux{[&] { return m_auxController.GetAxis(XBOX_RIGHT_TRIGGER, .5); }};//
  frc2::Button m_lTriggerAux{[&] { return m_auxController.GetAxis(XBOX_LEFT_TRIGGER, .5); }};//
  
  frc2::POVButton m_tDpadAux;
  frc2::POVButton m_bDpadAux;
  

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();
  void TrajectoryToCSV(frc::Trajectory trajectory);

  int cycle = 0;
  
  std::vector<SwerveDrivePathGenerator::finalPathPoint_t> m_path;
	

};
