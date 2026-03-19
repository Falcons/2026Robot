// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Turret.Shooter.Shooter;

public class FmsRumble extends SubsystemBase {
  private double timeOffset = 140.0;
  private CommandXboxController controllers[];
  private boolean teleStarted = false;
  private double rumbleTime = 130;
  private final double rumbleSecs = 2;

  private final Timer timer = new Timer();

  private final Shooter shooter;
  /** Creates a new FMS. */
  public FmsRumble(CommandXboxController controllers[], Shooter shooter) {
    SmartDashboard.putBoolean("FmsRumble/enableRumble", true);
    this.shooter = shooter;
    this.controllers = controllers;
  }

  @Override
  public void periodic() { 
    if (DriverStation.isDisabled() || !SmartDashboard.getBoolean("FmsRumble/enableRumble", true)) {
      setRumble(controllers, 0);
      return;
    }
    if (DriverStation.isTeleopEnabled() && !teleStarted){
      teleStarted = true;
      timer.reset();
      timer.start();
    }
    // if(shooter.atTargetRps()){
    //   setRumble(controllers, 0.1);
    // }else {
    //   setRumble(controllers, 0.1);
    // }

    // do +5 so it rumbles 5 seconds before
    if (getTime() <= rumbleTime + 2 && getTime() > rumbleTime) {
      setRumble(controllers, 0.5);
    }
    if (getTime() < rumbleTime) {
      rumbleTime -= 25; // each swap happens every 25 seconds
      setRumble(controllers, 0);
    }

    SmartDashboard.putNumber("FmsRumble/matchTime", getTime());
    SmartDashboard.putNumber("FmsRumble/rumbleTime", rumbleTime);
    SmartDashboard.putNumber("FmsRumble/time left", (int) getTime() - rumbleTime);
  }

  public double getTime() {
    return timeOffset - timer.get();
    // return DriverStation.getMatchTime();
  }

  public void setRumble(CommandXboxController xboxControllers[], double amount) {
    for (CommandXboxController controller : xboxControllers) {
      controller.getHID().setRumble(RumbleType.kBothRumble, amount);
    }
  }
}
