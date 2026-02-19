// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.Constants.IntakeConstants.RollersConstants;
import frc.robot.commands.AimHoodAndShoot;
import frc.robot.commands.Drive.TeleopDrive;
import frc.robot.commands.Hood.AutoHoodSim;
import frc.robot.commands.Hood.ManualHoodSim;
import frc.robot.commands.Intake.IntakeSim.PivotPidToggleSim;
import frc.robot.commands.Intake.IntakeSim.PivotShakeSim;
import frc.robot.commands.Turret.TurretSim.AutoTurretSim;
import frc.robot.commands.Turret.TurretSim.ManualTurretSim; // DONT REMOVE
import frc.robot.commands.Intake.PivotIntake;
import frc.robot.commands.Intake.PivotShake;
import frc.robot.commands.Intake.IntakeSim.PivotManualSim; // DONT REMOVE
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hood.HoodSim;
import frc.robot.subsystems.Intake.Pivot;
import frc.robot.subsystems.Intake.PivotSim;
import frc.robot.subsystems.Intake.Rollers;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.subsystems.Turret.Shooter;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretSim;

public class RobotContainer {

  // initialize robot's subsystems
  private final Swerve swerve = new Swerve();

  // simulated classes
  private final TurretSim turretSim = new TurretSim(swerve);
  private final HoodSim hoodSim = new HoodSim(swerve);
  private final PivotSim pivotSim = new PivotSim();

  // real classes
  private final Turret turret = new Turret(swerve);
  private final Shooter shooter = new Shooter(turret);
  private final Pivot pivot = new Pivot();
  private final Hood hood = new Hood(swerve);
  private final Rollers rollers = new Rollers();
  
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
      () -> MathUtil.applyDeadband(-driver.getLeftY(), ControllerConstants.deadBand), //() -> 0,
      () -> MathUtil.applyDeadband(-driver.getRightX(), ControllerConstants.deadBand), 
      () -> !driver.getHID().getLeftBumper()));

    // default hood to 0 and auto aim turret always
    turret.setDefaultCommand(Commands.run(turret::autoAim, turret));
    hood.setDefaultCommand(Commands.run(() -> hood.setHood(0), hood));
    
    // Configure the button bindings
    configureBindings();
    
    // Named Commands
    NamedCommands.registerCommand("Test", new InstantCommand(() -> System.out.println("test")));
    NamedCommands.registerCommand("AimAndShoot", new AimHoodAndShoot(hood, shooter));
    NamedCommands.registerCommand("Shake", new PivotShake(pivot));
    NamedCommands.registerCommand("Intake", new PivotIntake(pivot, rollers, PivotConstants.pivotOut, RollersConstants.rollerSpeed));
    NamedCommands.registerCommand("Outake", new PivotIntake(pivot, rollers, PivotConstants.pivotIn, 0));

    autoChooser = AutoBuilder.buildAutoChooser();

    // SIM CONTROLS:
    if (RobotBase.isReal()){return;}

    // auto aim
    turretSim.setDefaultCommand(new AutoTurretSim(turretSim));
    hoodSim.setDefaultCommand(new AutoHoodSim(hoodSim));
  }
  
  private void configureBindings() {
    
    driver.b().onTrue(new InstantCommand(swerve::zeroGyro));

    // SIM CONTROLS:
    if (RobotBase.isReal()){return;}

    // pivot toggle
    driver.a().onTrue(new PivotPidToggleSim(pivotSim));
    driver.x().onTrue(new PivotShakeSim(pivotSim));

    // manual turret
    // driver.axisMagnitudeGreaterThan(5, ControllerConstants.deadBand).whileTrue(
    //   new ManualTurretSim(
    //   turretSim, 
    //   () -> -driver.getRightY()));
    // manual pivot
    // driver.axisMagnitudeGreaterThan(5, ControllerConstants.deadBand).whileTrue(
    //   new PivotManualSim(
    //   pivotSim, 
    //   () -> driver.getRightY()));

    // manual hood
    driver.axisMagnitudeGreaterThan(5, ControllerConstants.deadBand).whileTrue(
      new ManualHoodSim(
      hoodSim, 
      () -> driver.getRightY()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}