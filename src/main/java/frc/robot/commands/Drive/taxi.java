// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class taxi extends Command {
  public Swerve swerve;
  public Timer timer;
  public double timeOutInSeconds;
  /**
   * creates a new taxi
   * @param swerve swerve subsystem
   * @param timeOutInSeconds drive timeout in seconds
   */
  public taxi(Swerve swerve, double timeOutInSeconds) {
    this.swerve = swerve;
    this.timeOutInSeconds = timeOutInSeconds;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(this.getName() + " start");
    timer = new Timer();
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d translation = new Translation2d(1, 0);
    swerve.drive(translation, 0.0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(this.getName() + " end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(timeOutInSeconds);
  }
}
