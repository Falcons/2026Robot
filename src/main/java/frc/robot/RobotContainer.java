// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.Test;
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

    // // add default commands (run when no other commands are running)
    // swerve.setDefaultCommand(new TeleopDrive( 
    //   swerve, 
    //   () -> MathUtil.applyDeadband(-driver.getLeftX(), ControllerConstants.deadBand), 
    //   () -> MathUtil.applyDeadband(-driver.getLeftY(), ControllerConstants.deadBand), 
    //   () -> MathUtil.applyDeadband(-driver.getRightX(), ControllerConstants.deadBand), 
    //   () -> !driver.getHID().getLeftBumper()));

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = swerve.driveCommand(
        () -> MathUtil.applyDeadband(-driver.getLeftY(), ControllerConstants.deadBand),
        () -> MathUtil.applyDeadband(-driver.getLeftX(), ControllerConstants.deadBand),
        () -> -driver.getRightX(),
        () -> -driver.getRightY());
        
    swerve.setDefaultCommand(driveFieldOrientedDirectAngle);
    // Configure the button bindings
    configureBindings();

    // make commands for auto
    NamedCommands.registerCommand("Test", new Test());


    // auto load all pathplanner autos
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Choose Auto: ", autoChooser);
  }
  
  private void configureBindings() {
    driver.b().onTrue(new ZeroGyro(swerve));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}