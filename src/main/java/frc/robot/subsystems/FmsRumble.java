// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class FmsRumble extends SubsystemBase {
  private double timeOffset = 130.0;
  private CommandXboxController controllers[];
  /** Creates a new FMS. */
  public FmsRumble(CommandXboxController controllers[]) {
    this.controllers = controllers;
  }

  @Override
  public void periodic() { 
    // do +5 so it rumbles 5 seconds before
    if (DriverStation.getMatchTime() <= timeOffset + 5 && DriverStation.getMatchTime() >= timeOffset) {
      setRumble(controllers, 0.5);
    }
    if (DriverStation.getMatchTime() == timeOffset) {
      timeOffset -= 25; // each swap happens every 25 seconds
      setRumble(controllers, 0);
    }
    SmartDashboard.putNumber("FmsRumble/matchTime", DriverStation.getMatchTime());
  }

  public void setRumble(CommandXboxController xboxControllers[], double amount) {
    for (CommandXboxController controller : xboxControllers) {
      controller.getHID().setRumble(RumbleType.kBothRumble, amount);
    }
  }
}
