/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "components/Joystick2481.h"
#include "subsystems/DriveSubsystem.h"
#include "components/XboxController2481.h"
#include <units/units.h>
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveWithJoystickCommand
    : public frc2::CommandHelper<frc2::CommandBase, DriveWithJoystickCommand> {
 public:
  DriveWithJoystickCommand(DriveSubsystem* swerveDrivetrain, Joystick2481* driverController){
    m_pDrivetrain = swerveDrivetrain;
    m_pDriverController = driverController;
    AddRequirements(m_pDrivetrain);
  }

  void Initialize() override{
    
  }

  void Execute() override{
    m_fieldCentric = m_pDrivetrain->getFiedCentricForJoystick();
    // adjust to max speeds
    double base = .01;
    units::radians_per_second_t rot = units::radians_per_second_t(-m_pDriverController->GetRawAxis(XBOX_RIGHT_X_AXIS));
    if(fabs(rot.to<double>()) > base){
      rot = rot * 2.2;
    }
    if(m_fieldCentric){
      m_pDrivetrain->Drive(units::meters_per_second_t(-m_pDriverController->GetRawAxis(XBOX_LEFT_Y_AXIS)),
                         units::meters_per_second_t(-m_pDriverController->GetRawAxis(XBOX_LEFT_X_AXIS)),
                         rot,
                         m_fieldCentric
                         );
    }else{
      m_pDrivetrain->Drive(units::meters_per_second_t(-m_pDriverController->GetRawAxis(XBOX_LEFT_Y_AXIS)),
                         units::meters_per_second_t(-m_pDriverController->GetRawAxis(XBOX_LEFT_X_AXIS)),
                         rot,
                         m_fieldCentric
                         );
    }
    
  }

  void End(bool interrupted) override{
    m_pDrivetrain->stop();
  }

  bool IsFinished() override{
    return false;
  }
  void ToggleFeildCentric(){
    if(m_fieldCentric){
      m_fieldCentric = false;
    }else{
      m_fieldCentric = true;
    }
  }

 private:
    Joystick2481* m_pDriverController;
    DriveSubsystem* m_pDrivetrain;
    bool m_fieldCentric = false;
};
