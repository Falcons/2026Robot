// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Shooter.ShooterSim;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootSim extends Command {
  private final ShooterSim shooterSim;
  /** Creates a new Shoot. */
  public ShootSim(ShooterSim shooterSim) {
    this.shooterSim = shooterSim;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSim);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(getName() + " start");
    shooterSim.shooterRunning = false;
    shooterSim.shotTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSim.shootWhenMaxSpeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSim.shooterRunning = false;
    shooterSim.shotTimer.stop();
    shooterSim.shotTimer.reset();
    System.out.println(interrupted ? getName()+ " interrupted" : getName() + " end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterSim.shooterRPMlow();
  }
}
