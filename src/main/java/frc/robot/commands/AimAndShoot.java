// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.Shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimAndShoot extends ParallelDeadlineGroup {
  /** Creates a new AimHoodAndShoot. */
  public AimAndShoot(Hood hood, Shooter shooter, Turret turret, Double shootTime) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(Commands.runEnd(shooter::autoShoot, () -> shooter.setShooter(0.0), shooter).withTimeout(shootTime));
    addCommands(Commands.run(hood::autoAim, hood), Commands.run(turret::autoAim));
  }
}
