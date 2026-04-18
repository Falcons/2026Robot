// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.IntakeConstants.RollersConstants;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Intake.Rollers;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.Shooter.Shooter;
import frc.robot.subsystems.Turret.Shooter.Transfer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShoot extends ParallelCommandGroup {
  /** Creates a new AutoShoot. */
  public AutoShoot(Turret turret, Hood hood, Transfer transfer, Shooter shooter, Rollers rollers) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PrintCommand("auto shoot started"),
      Commands.runEnd(turret::autoAim, turret::stop, turret),
      Commands.runEnd(hood::autoAim, () -> hood.set(0.0), hood),
      Commands.runEnd(shooter::autoShoot, shooter::stopShooter, shooter),
      Commands.runEnd(transfer::autoTransfer, transfer::stop, transfer),
      // Commands.runEnd(transfer::autoPulseTransfer, transfer::stop, transfer),
      // Commands.runEnd(() -> rollers.setRPS(RollersConstants.rollerSpeedRPS), rollers::stop, rollers)
      Commands.runEnd(() -> rollers.set(RollersConstants.rollerSpeed), rollers::stop, rollers)
    );
  }
}
