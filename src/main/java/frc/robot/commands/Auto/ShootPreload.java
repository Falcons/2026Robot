// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.commands.Intake.PivotPid;
import frc.robot.commands.Turret.AutoShoot;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Intake.Pivot;
import frc.robot.subsystems.Intake.Rollers;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.Shooter.Shooter;
import frc.robot.subsystems.Turret.Shooter.Transfer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootPreload extends ParallelCommandGroup {
  /** Creates a new ShootPreload. */
  public ShootPreload(Turret turret, Hood hood, Transfer transfer, Shooter shooter, Rollers rollers, Pivot pivot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PivotPid(pivot, PivotConstants.pivotOut).withTimeout(1).asProxy(),
      new AutoShoot(turret, hood, transfer, shooter, rollers).withTimeout(10).andThen(Commands.print("autoShoot end")),
      new SequentialCommandGroup(
        new WaitCommand(1),
        new PivotPid(pivot, PivotConstants.pivotIn).withTimeout(1),
        new WaitCommand(1).asProxy(),
        new PivotPid(pivot, PivotConstants.pivotOut).withTimeout(1))
    );
  }
}
