// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret.TurretSim;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.TurretSim.MovementSim;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoTurretSim extends Command {

  private final MovementSim movementSim;
  /** Creates a new ManualPivotSim. */
  public AutoTurretSim(MovementSim movementSim) {
    this.movementSim = movementSim;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(movementSim);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(this.getName() + " start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // movementSim.setPivot(speedX.getAsDouble());
    movementSim.autoAim();
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
