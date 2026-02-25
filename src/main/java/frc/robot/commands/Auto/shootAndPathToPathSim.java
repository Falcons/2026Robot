// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AimHoodAndShootSim;
import frc.robot.subsystems.Hood.HoodSim;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class shootAndPathToPathSim extends ParallelCommandGroup {
  /** Creates a new shootAndPathToPathSim. */
  public shootAndPathToPathSim(PathPlannerPath goalPath, HoodSim hood) {
    addCommands(
      AutoBuilder.pathfindThenFollowPath(goalPath, DriveConstants.pathFindingConstraints),
      new AimHoodAndShootSim(hood)
    );
  }
}
