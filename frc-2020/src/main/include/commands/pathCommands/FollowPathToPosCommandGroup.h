// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// #pragma once

// #include <frc2/command/CommandHelper.h>
// #include <frc2/command/ParallelRaceGroup.h>
// #include "Utils/SwerveDrivePathFollower.h"
// #include "subsystems/DriveSubsystem.h"
// #include <frc/geometry/Pose2d.h>
// #include "commands/pathCommands/WaitForPosCommand.h"
// #include "commands/pathCommands/PathFollowerCommand.h"

// class FollowPathToPosCommandGroup
//     : public frc2::CommandHelper<frc2::ParallelRaceGroup,
//                                  FollowPathToPosCommandGroup> {
//  public:
//   FollowPathToPosCommandGroup(SwerveDrivePathFollower* follower, DriveSubsystem* driveSubsystem, double xPos, double yPos, double x, double y){
//     AddCommands(
//       WaitForPosCommand(driveSubsystem, xPos, yPos, x, y),
//       PathFollowerCommand(follower, driveSubsystem)
//     );
//   }
// };
