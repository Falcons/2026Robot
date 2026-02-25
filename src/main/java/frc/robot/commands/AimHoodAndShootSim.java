// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Hood.HoodSim;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimHoodAndShootSim extends ParallelDeadlineGroup {
  /** Creates a new AimHoodAndShootSim. */
  public AimHoodAndShootSim(HoodSim hoodSim) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new PrintCommand("pretend this is shooting").andThen(new WaitCommand(1)));//TODO: get time to slow down
    addCommands(Commands.run(hoodSim::autoAim, hoodSim));
  }
}
