// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.commands.Intake.PivotPid;
import frc.robot.subsystems.Intake.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeShake extends SequentialCommandGroup {
  /** Creates a new IntakeShake. */
  public IntakeShake(Pivot pivot, double delay) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PrintCommand("intake shake start: " + delay),
      new WaitCommand(delay),
      new PivotPid(pivot, PivotConstants.pivotIn).withTimeout(0.5),
      new PivotPid(pivot, PivotConstants.pivotOut).withTimeout(0.5)
      );
  }
}
