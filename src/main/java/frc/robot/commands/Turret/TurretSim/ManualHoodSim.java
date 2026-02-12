// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret.TurretSim;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.TurretSim.MovementSim;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualHoodSim extends Command {
  private final MovementSim movementSim;
  private DoubleSupplier speed;
  /** Creates a new ManualHood. */
  public ManualHoodSim(MovementSim movementSim, DoubleSupplier speed) {
    this.movementSim = movementSim;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(movementSim);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    movementSim.setHood(speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
