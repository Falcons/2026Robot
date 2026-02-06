// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake.IntakeSim;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.PivotSim;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotManualSim extends Command {
  private final PivotSim pivotSim;
  private final DoubleSupplier speed;
  /** Creates a new PivotControlSim. */
  public PivotManualSim(PivotSim pivotSim, DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivotSim = pivotSim;
    this.speed = speed;
    addRequirements(pivotSim);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotSim.setPivot(speed.getAsDouble() * 0.02); //delta time
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
