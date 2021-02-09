// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// #pragma once

// #include <frc2/command/InstantCommand.h>
// #include <frc2/command/CommandHelper.h>
// #include "Utils/SwerveDrivePathFollower.h"
// #include "subsystems/DriveSubsystem.h"
// /**
//  * An example command.
//  *
//  * <p>Note that this extends CommandHelper, rather extending CommandBase
//  * directly; this is crucially important, or else the decorator functions in
//  * Command will *not* work!
//  */
// class StartPathFollowing
//     : public frc2::CommandHelper<frc2::InstantCommand, StartPathFollowing> {
//  private:
//   SwerveDrivePathFollower* m_pFollower;
//   DriveSubsystem* m_pDriveSubsystem;
//   bool m_zero;
//  public:
//   StartPathFollowing(SwerveDrivePathFollower* follower, DriveSubsystem* driveSubsystem, std::vector<SwerveDrivePathGenerator::waypoint_t> &waypoints, bool zero = false){
//     m_pFollower = follower;
//     m_pDriveSubsystem = driveSubsystem;
//     AddRequirements(m_pDriveSubsystem);
//     m_pFollower->generatePath(waypoints);//if meters
//     m_zero = zero;
//   }

//   void Initialize() override{
//     m_pFollower->start();
//     if(m_zero){
//       m_pDriveSubsystem->ResetOdometry(m_pFollower->getPointPos(0));
//     }
		
//   }

// };
