// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Turret.Shooter.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ControllerRumbleAtSpeed extends Command {
  private final Shooter shooter;
  private final CommandXboxController controller;

  /** Creates a new Shoot. */
  public ControllerRumbleAtSpeed(Shooter shooter, CommandXboxController controller) {
    this.shooter = shooter;
    this.controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.getShooterRealSpeed() >= shooter.getShooterSetSpeed()-0.1 && shooter.getShooterRealSpeed() <= shooter.getShooterSetSpeed()+0.1){
      controller.setRumble(RumbleType.kLeftRumble, 0.3);
      controller.setRumble(RumbleType.kRightRumble, 0.3);
    }else {
        controller.setRumble(RumbleType.kLeftRumble, 0);
        controller.setRumble(RumbleType.kRightRumble, 0);
      }
  }


  @Override
  public boolean runsWhenDisabled(){
      return true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}