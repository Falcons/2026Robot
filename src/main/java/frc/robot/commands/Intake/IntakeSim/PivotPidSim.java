// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake.IntakeSim;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.PivotSim;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotPidSim extends Command {
  PivotSim pivotSim;
  double setpoint;
  /** Creates a new PivotPid. */
  public PivotPidSim(PivotSim pivotSim, double setpoint) {
    this.pivotSim = pivotSim;
    this.setpoint = setpoint;
    addRequirements(pivotSim);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(this.getName() + " starts");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotSim.setPivotPid(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(this.getName() + " ends");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivotSim.atSetpoint();
  }
}
