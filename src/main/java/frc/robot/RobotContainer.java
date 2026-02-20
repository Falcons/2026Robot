// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.Constants.IntakeConstants.RollersConstants;
import frc.robot.Constants.TurretConstants.ShooterConstants;
import frc.robot.commands.AimHoodAndShoot;
import frc.robot.commands.AimHoodAndShootSim;
import frc.robot.commands.Drive.TeleopDrive;
import frc.robot.commands.Drive.taxi;
import frc.robot.commands.Hood.AutoHoodSim;
import frc.robot.commands.Hood.ManualHoodSim;
import frc.robot.commands.Intake.IntakeSim.PivotPidToggleSim;
import frc.robot.commands.Intake.IntakeSim.PivotShakeSim;
import frc.robot.commands.Turret.ManualTurret;
import frc.robot.commands.Turret.TurretSim.AutoTurretSim;
import frc.robot.commands.Turret.TurretSim.ManualTurretSim; // DONT REMOVE
import frc.robot.commands.Intake.PivotIntake;
import frc.robot.commands.Intake.PivotPid;
import frc.robot.commands.Intake.PivotShake;
import frc.robot.commands.Intake.IntakeSim.PivotManualSim; // DONT REMOVE
import frc.robot.commands.Intake.IntakeSim.PivotPidSim;
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
  private TurretSim turretSim;
  private HoodSim hoodSim;;
  private PivotSim pivotSim;;

  // real classes
  private Turret turret;
  private Shooter shooter;
  private Pivot pivot;
  private Hood hood;
  private Rollers rollers;
  
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // make a chooser option to select autos
  SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  // variables

  public RobotContainer() {
    if(Robot.isReal()){
      setupReal();
      configureRealBindings();
    }else {
      setupSim();
      configureSimBindings();

      // DRIVER
    driver.b().onTrue(new InstantCommand(swerve::zeroGyro));
    // slow mode toggle
    driver.start().toggleOnTrue(new InstantCommand(
      () -> swerve.setMaxAllowableSpeed(DriveConstants.slowModeMPS, DriveConstants.slowModeRPS)));
    driver.start().toggleOnFalse(new InstantCommand(
      () -> swerve.setMaxAllowableSpeed(swerve.getMaximumVelocity(), swerve.getMaximumAngularVelocity())));
    // slow mode hold
    driver.rightBumper().whileTrue(Commands.startEnd(
      () -> swerve.setMaxAllowableSpeed(DriveConstants.slowModeMPS, DriveConstants.slowModeRPS), 
      () -> swerve.setMaxAllowableSpeed(swerve.getMaximumVelocity(), swerve.getMaximumAngularVelocity()), swerve));
    }
    
    // add default commands (run when no other commands are running)
    swerve.setDefaultCommand(new TeleopDrive( 
      swerve, 
      () -> MathUtil.applyDeadband(-driver.getLeftX(), ControllerConstants.deadBand), 
      () -> MathUtil.applyDeadband(-driver.getLeftY(), ControllerConstants.deadBand),
      () -> MathUtil.applyDeadband(-driver.getRightX(), ControllerConstants.deadBand), 
      () -> !driver.getHID().getLeftBumper()));

    boolean isCompetition = SmartDashboard.getBoolean("Input/is competition", false);
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
        ? stream.filter(auto -> !auto.getName().startsWith("Test"))
        : stream);

    SmartDashboard.putData("auto Chooser" ,autoChooser);
    autoChooser.addOption("timeout left", AutoBuilder.pathfindToPose(DriveConstants.timeoutPoseLeft, DriveConstants.pathFindingConstraints));
    autoChooser.addOption("timeout right", AutoBuilder.pathfindToPose(DriveConstants.timeoutPoseRight, DriveConstants.pathFindingConstraints));
  }
  
  private void setupReal() {
    //initializing real classes
    this.turret = new Turret(swerve);
    this.shooter = new Shooter(turret);
    this.pivot = new Pivot();
    this.hood = new Hood(swerve);
    this.rollers = new Rollers();
    
    // default hood to 0 and auto aim turret always
    turret.setDefaultCommand(Commands.run(turret::autoAim, turret));
    hood.setDefaultCommand(Commands.run(() -> hood.setHood(0), hood));

    // Named Commands
    NamedCommands.registerCommand("Test", new InstantCommand(() -> System.out.println("test")));
    NamedCommands.registerCommand("AimAndShoot", new AimHoodAndShoot(hood, shooter));
    NamedCommands.registerCommand("Shake", new PivotShake(pivot));
    NamedCommands.registerCommand("Intake", new PivotIntake(pivot, rollers, PivotConstants.pivotOut, RollersConstants.rollerSpeed));
    NamedCommands.registerCommand("Outake", new PivotIntake(pivot, rollers, PivotConstants.pivotIn, 0).withTimeout(1));
  }

  private void configureRealBindings() {
    // DRIVER (secondary controls)
    // rollers
    driver.axisMagnitudeGreaterThan(3, ControllerConstants.triggerDeadBand).whileTrue(
      Commands.run(() -> rollers.set(RollersConstants.rollerSpeed)));

    // charge fire
    driver.axisMagnitudeGreaterThan(4, ControllerConstants.triggerDeadBand).whileTrue(
      Commands.run(() -> shooter.setShooterWithkicker(driver::getRightTriggerAxis, 
      ShooterConstants.maxKickerSpeed), shooter));
    
    // intake out/in, shhake
    driver.povUp().onTrue(new PivotPid(pivot, PivotConstants.pivotOut));
    driver.povDown().onTrue(new PivotPid(pivot, PivotConstants.pivotIn));
    driver.povLeft().onTrue(new PivotShake(pivot));

    // OPERATOR
    // move transfer backwards
    operator.leftBumper().whileTrue(Commands.run(() -> shooter.setTransfer(-ShooterConstants.maxTransferSpeed), shooter));

    // spin shooter
    operator.axisMagnitudeGreaterThan(3, ControllerConstants.triggerDeadBand).whileTrue(
      Commands.run(() -> shooter.setShooterWithkicker(driver::getLeftTriggerAxis, 
      ShooterConstants.maxKickerSpeed), shooter));

    // actually shoot
    operator.axisMagnitudeGreaterThan(4, ControllerConstants.triggerDeadBand).whileTrue(
      Commands.run(() -> shooter.fullShoot(driver::getRightTriggerAxis, 
      ShooterConstants.maxTransferSpeed, 
      ShooterConstants.maxKickerSpeed), shooter));

    // manual turret
    operator.axisMagnitudeGreaterThan(1, ControllerConstants.deadBand).whileTrue(
      new ManualTurret(turret, operator::getLeftX));
      
    // manual hood
    operator.axisMagnitudeGreaterThan(2, ControllerConstants.deadBand).whileTrue(
      Commands.run(() -> hood.moveHood(operator::getLeftY), hood));

    // main fire
    operator.b().whileTrue(
      Commands.runEnd(shooter::shootWhenMaxSpeed, () -> shooter.shooterRunning = false, shooter));

    // spin intake - rollers 
    operator.x().whileTrue(Commands.run(() -> rollers.set(RollersConstants.rollerSpeed)));
    operator.povRight().whileTrue(Commands.run(() -> rollers.set(-RollersConstants.rollerSpeed)));

    // intake out and in, and shake
    operator.povUp().onTrue(new PivotPid(pivot, PivotConstants.pivotOut));
    operator.povDown().onTrue(new PivotPid(pivot, PivotConstants.pivotIn));
    operator.povLeft().onTrue(new PivotShake(pivot));
  }
 
  private void setupSim() {
    // initializing sim  classes
    this.turretSim = new TurretSim(swerve);
    this.hoodSim = new HoodSim(swerve);
    this.pivotSim = new PivotSim();

    // silence joystick warning
    DriverStation.silenceJoystickConnectionWarning(true);

    // Named Commands
    NamedCommands.registerCommand("Test", new InstantCommand(() -> System.out.println("test")));
    NamedCommands.registerCommand("AimAndShoot", new AimHoodAndShootSim(hoodSim));
    NamedCommands.registerCommand("Shake", new PivotShakeSim(pivotSim));
    NamedCommands.registerCommand("Intake", new PivotPidSim(pivotSim, PivotConstants.pivotOut));
    NamedCommands.registerCommand("Outake", new PivotPidSim(pivotSim, PivotConstants.pivotIn));

    // auto aim
    turretSim.setDefaultCommand(new AutoTurretSim(turretSim));
    hoodSim.setDefaultCommand(new AutoHoodSim(hoodSim));
  }

  private void configureSimBindings(){
    // pivot toggle
    driver.a().onTrue(new PivotPidToggleSim(pivotSim));
    driver.x().onTrue(new PivotShakeSim(pivotSim));

    // manual turret
    /* driver.axisMagnitudeGreaterThan(5, ControllerConstants.deadBand).whileTrue(
       new ManualTurretSim(
       turretSim, 
       () -> -driver.getRightY()));
     manual pivot
     driver.axisMagnitudeGreaterThan(5, ControllerConstants.deadBand).whileTrue(
       new PivotManualSim(
       pivotSim, 
       () -> driver.getRightY())); */

    // manual hood
    driver.axisMagnitudeGreaterThan(5, ControllerConstants.deadBand).whileTrue(
      new ManualHoodSim(
      hoodSim, 
      () -> driver.getRightY()));
  }
  
  public Command getAutonomousCommand() {
    try{
      return autoChooser.getSelected();
    }catch (Exception err){
      System.err.println("error loading autonomous command | " + err);
      return new taxi(swerve, 1);
    }
  }
}