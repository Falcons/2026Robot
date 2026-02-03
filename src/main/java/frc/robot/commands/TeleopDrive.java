// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopDrive extends Command {
  /** Creates a new TeleopDrivee. */
  private final SwerveSubsystem swerve;
  private final DoubleSupplier  controlX, controlY, rot;
  private final SlewRateLimiter xSlewLimiter = new SlewRateLimiter(0.5);
  private final SlewRateLimiter ySlewLimiter = new SlewRateLimiter(0.5);
  private final BooleanSupplier fieldRelative;

  public TeleopDrive(SwerveSubsystem swerve, DoubleSupplier controlX, DoubleSupplier controlY, DoubleSupplier rot, BooleanSupplier fieldRelative) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.controlX = controlX;
    this.controlY = controlY;
    this.rot = rot;
    this.fieldRelative = fieldRelative;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double limitedXControl = xSlewLimiter.calculate(controlX.getAsDouble());
    double limitedYcontrol = ySlewLimiter.calculate(controlY.getAsDouble());
    swerve.drive(new Translation2d(
      limitedXControl  * swerve.getMaximumVelocity(),
      limitedYcontrol  * swerve.getMaximumVelocity()),
      rot.getAsDouble() * swerve.getMaximumAngularVelocity(),
      fieldRelative.getAsBoolean());
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
