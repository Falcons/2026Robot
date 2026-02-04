// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.Drive.TeleopDrive;
import frc.robot.commands.Drive.ZeroGyro;
import frc.robot.subsystems.Swerve.Swerve;

public class RobotContainer {

  // initialize robot's subsystems
  private final Swerve swerve = new Swerve();

  private final CommandXboxController driver = new CommandXboxController(0);

  // make a chooser option to select autos
  SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  // variables

  public RobotContainer() {

    // silence joystick warning if simulation
    if (Robot.isSimulation()){DriverStation.silenceJoystickConnectionWarning(true);}

    // add default commands (run when no other commands are running)
    swerve.setDefaultCommand(new TeleopDrive( 
      swerve, 
      () -> MathUtil.applyDeadband(-driver.getLeftX(), ControllerConstants.deadBand), 
      () -> MathUtil.applyDeadband(-driver.getLeftY(), ControllerConstants.deadBand), 
      () -> MathUtil.applyDeadband(-driver.getRightX(), ControllerConstants.deadBand), 
      () -> !driver.getHID().getLeftBumper()));

    // Configure the button bindings
    configureBindings();
  }
  
  private void configureBindings() {
    driver.b().onTrue(new ZeroGyro(swerve));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}


/* 
Austin
TODO: Turret movement

========================
Riyan
TODO: Turret shooter

========================
Likith
TODO: pathplanner

Intake rollers

========================
Justin
TODO: 

intake pivot
Intake rollers

========================
Abid
TODO: 

Intake rollers

========================
Reena
TODO: Climb


========================
May
TODO: Climb

*/