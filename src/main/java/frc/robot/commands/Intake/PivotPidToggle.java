// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.subsystems.Intake.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotPidToggle extends Command {
  Pivot pivot;
  double pivotPosition;
  /** Creates a new PivotPidToggle. */
  public PivotPidToggle(Pivot pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivot = pivot;
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(this.getName() + " start");
    pivotPosition = PivotConstants.pivotMin;
    if (pivot.getDegrees() >= PivotConstants.pivotMax - 10){pivotPosition = PivotConstants.pivotMax;}
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Pivot/home", pivotPosition);
    pivot.setPivotPid(PivotConstants.pivotMax);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(this.getName() + " end");
    pivot.stopPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivot.atSetpoint();
  }
}
