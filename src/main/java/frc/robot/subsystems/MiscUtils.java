// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve.Swerve;

public class MiscUtils extends SubsystemBase {
  SendableChooser<Command> autoChooser;
  Swerve swerve;
  /** Creates a new MiscUtils. */
  public MiscUtils(SendableChooser<Command> autoChooser, Swerve swerve) {
    this.autoChooser = autoChooser;
    this.swerve = swerve;

    if(Constants.logDashboard) DataLogManager.start();
  }

  @Override
  public void periodic() {
    Command SelectedAuto = autoChooser.getSelected();
    PathPlannerAuto SelectedPathPlannerAuto = new PathPlannerAuto(SelectedAuto.getName());
    
  }
}
