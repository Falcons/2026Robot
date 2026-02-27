// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AimAndShootSim;
import frc.robot.subsystems.Hood.HoodSim;
import frc.robot.subsystems.Turret.ShooterSim;
import frc.robot.subsystems.Turret.TurretSim;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class shootAndPathToPoseSim extends ParallelCommandGroup {
  /** Creates a new shootAndPathFindSim. */
  public shootAndPathToPoseSim(Pose2d endPose, HoodSim hood, TurretSim turret, ShooterSim shooterSim) {
    addCommands(
      AutoBuilder.pathfindToPose(endPose, DriveConstants.pathFindingConstraints),
      new AimAndShootSim(hood, turret, shooterSim)
    );
  }
}
