// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Movement;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualTurret extends Command {
  private final Movement movement;
  private DoubleSupplier speed;
  /** Creates a new ManualAim. */
  public ManualTurret(Movement movement, DoubleSupplier speed) {
    this.movement = movement;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(movement);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(this.getName() + " start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    movement.setTurret(speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(this.getName() + " end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
