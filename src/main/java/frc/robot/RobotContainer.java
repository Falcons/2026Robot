// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.Constants.IntakeConstants.RollersConstants;
import frc.robot.Constants.TurretConstants.ShooterConstants;
import frc.robot.Util.AllianceFlipUtil;
import frc.robot.commands.AimAndShoot;
import frc.robot.commands.AimAndShootSim;
import frc.robot.commands.Auto.shootAndPathToPathSim;
import frc.robot.commands.Auto.shootAndPathToPoseSim;
import frc.robot.commands.Drive.TeleopDrive;
import frc.robot.commands.Drive.taxi;
import frc.robot.commands.Hood.ManualHoodSim;
import frc.robot.commands.Intake.IntakeSim.PivotPidToggleSim;
import frc.robot.commands.Intake.IntakeSim.PivotShakeSim;
import frc.robot.commands.Turret.ShootSim; 
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
import frc.robot.subsystems.Intake.RollersSim;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.subsystems.Turret.Shooter;
import frc.robot.subsystems.Turret.ShooterSim;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretSim;

public class RobotContainer {

  // initialize robot's subsystems
  private Swerve swerve = new Swerve(); 

  // simulated classes
  private TurretSim turretSim;
  private ShooterSim shooterSim;
  private HoodSim hoodSim;;
  private PivotSim pivotSim;
  private RollersSim rollersSim;

  // real classes
  private Turret turret;
  private Shooter shooter;
  private Pivot pivot;
  private Hood hood;
  private Rollers rollers;
  
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // make a chooser option to select autos
  public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

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
  }
  
  private void setupReal() {
    //initializing real classes
    this.turret = new Turret(swerve, shooter);
    this.shooter = new Shooter(turret);
    this.rollers = new Rollers();
    this.pivot = new Pivot(rollers );
    this.hood = new Hood(swerve, shooter);
    
    // default hood to 0 and auto aim turret always
    // turret.setDefaultCommand(Commands.run(turret::autoAim, turret));
    // hood.setDefaultCommand(Commands.run(() -> hood.setDeg(0), hood));

    pivot.setDefaultCommand(Commands.runEnd(() -> pivot.setPivot(operator::getRightY), () -> pivot.setPivot(0), pivot));

    // Named Commands
    NamedCommands.registerCommand("Test", new PrintCommand("test"));
    NamedCommands.registerCommand("AimAndShoot", new AimAndShoot(hood, shooter, turret));
    NamedCommands.registerCommand("Shake", new PivotShake(pivot));
    NamedCommands.registerCommand("Intake", new PivotIntake(pivot, rollers, PivotConstants.pivotOut, RollersConstants.rollerSpeed));
    NamedCommands.registerCommand("Outake", new PivotIntake(pivot, rollers, PivotConstants.pivotIn, 0).withTimeout(1));
    
    //autos
     DriverStation.waitForDsConnection(0);

    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> Constants.isCompetition
        ? stream.filter(auto -> !auto.getName().startsWith("Test")): stream);
    autoChooser.addOption("timeout left", AutoBuilder.pathfindToPose(AllianceFlipUtil.apply(DriveConstants.timeoutPoseLeft), DriveConstants.pathFindingConstraints));
    autoChooser.addOption("timeout right", AutoBuilder.pathfindToPose(AllianceFlipUtil.apply(DriveConstants.timeoutPoseRight), DriveConstants.pathFindingConstraints));
    autoChooser.addOption("preload shoot left", Commands.parallel(
      AutoBuilder.pathfindToPose(AllianceFlipUtil.apply(DriveConstants.ShootingStartLeft), DriveConstants.pathFindingConstraints),
      new AimAndShoot(hood, shooter, turret)));
    autoChooser.addOption("preload shoot right", Commands.parallel(
      AutoBuilder.pathfindToPose(AllianceFlipUtil.apply(DriveConstants.ShootingStartRight), DriveConstants.pathFindingConstraints),
      new AimAndShoot(hood, shooter, turret)));
    
    SmartDashboard.putData("auto Chooser" ,autoChooser);
  }

  private void configureRealBindings() {
    // DRIVER (secondary controls)
    // rollers
    // driver.axisMagnitudeGreaterThan(3, ControllerConstants.triggerDeadBand).whileTrue(
    //   Commands.run(() -> rollers.set(RollersConstants.rollerSpeed)));

    // // charge fire
    // driver.axisMagnitudeGreaterThan(4, ControllerConstants.triggerDeadBand).whileTrue(
    //   Commands.run(() -> shooter.setShooterWithkicker(driver::getRightTriggerAxis, 
    //   ShooterConstants.maxKickerSpeed), shooter));

    // intake out/in, shake
    // driver.povUp().onTrue(new PivotPid(pivot, PivotConstants.pivotOut));
    // driver.povDown().onTrue(new PivotPid(pivot, PivotConstants.pivotIn));
    // driver.povLeft().onTrue(new PivotShake(pivot));

    // OPERATOR
    // move transfer
    operator.rightTrigger().whileTrue(Commands.runEnd(shooter::pulseTransfer, () -> shooter.setTransfer(0), shooter));
    operator.leftBumper().whileTrue(Commands.run(() -> shooter.setTransfer(-ShooterConstants.maxTransferSpeed), shooter));

    // // spin shooter
    operator.axisMagnitudeGreaterThan(2, ControllerConstants.triggerDeadBand).whileTrue(
      Commands.runEnd(() -> shooter.setShooter(operator::getLeftTriggerAxis), () -> shooter.setShooter(0.0), shooter));

    // // actually shoot
    // operator.axisMagnitudeGreaterThan(4, ControllerConstants.triggerDeadBand).whileTrue(
    //   Commands.run(() -> shooter.fullShoot(driver::getRightTriggerAxis, 
    //   ShooterConstants.maxTransferSpeed, 
    //   ShooterConstants.maxKickerSpeed), shooter));

    // // manual turret
    // operator.axisMagnitudeGreaterThan(0, ControllerConstants.deadBand).whileTrue(
    //   Commands.run(() -> turret.set(() -> operator.getLeftX()), turret));
      
    // // manual hood
    // operator.axisMagnitudeGreaterThan(2, ControllerConstants.deadBand).whileTrue(
    //   Commands.run(() -> hood.moveHood(operator::getLeftY), hood));

    // main fire
    // operator.b().whileTrue(
    //   Commands.runEnd(shooter::shootWhenMaxSpeed, () -> shooter.shooterRunning = false, shooter));

    // spin intake - rollers 
    operator.x().whileTrue(Commands.runEnd(() -> rollers.set(RollersConstants.rollerSpeed), () -> rollers.set(0), rollers));
    operator.povRight().whileTrue(Commands.runEnd(() -> rollers.set(-RollersConstants.rollerSpeed), () -> rollers.set(0), rollers));

    // intake out and in, and shake
    operator.povUp().onTrue(new PivotPid(pivot, PivotConstants.pivotOut));
    operator.povDown().onTrue(new PivotPid(pivot, PivotConstants.pivotIn));
    operator.povLeft().onTrue(new PivotShake(pivot));
    
    // driver.y().whileTrue(Commands.runEnd(() -> shooter.playSong("src/main/deploy/chirp/crazy_train.chrp"), shooter::stopSong)); //music bs
      
  }
 
  private void setupSim(){
    // initializing sim  classes

    // need to do null shenangigans since they all depend on each other ._.
    this.turretSim = new TurretSim(swerve, null); 
    this.shooterSim = new ShooterSim(turretSim);
    this.turretSim.setShooterSim(shooterSim);

    this.hoodSim = new HoodSim(swerve, shooterSim);
    this.rollersSim = new RollersSim();
    this.pivotSim = new PivotSim(rollersSim);

    // silence joystick warning
    DriverStation.silenceJoystickConnectionWarning(true);

    // Named Commands
    NamedCommands.registerCommand("Test", new PrintCommand("test"));
    NamedCommands.registerCommand("AimAndShoot", new AimAndShootSim(hoodSim, turretSim, shooterSim));
    NamedCommands.registerCommand("Shake", new PivotShakeSim(pivotSim));
    NamedCommands.registerCommand("Intake", new PivotPidSim(pivotSim, PivotConstants.pivotOut));
    NamedCommands.registerCommand("Outake", new PivotPidSim(pivotSim, PivotConstants.pivotIn));

    // pathplaner events
    new EventTrigger("Intake").onTrue(new PivotPidSim(pivotSim, PivotConstants.pivotOut));
    new EventTrigger("Outake").onTrue(new PivotPidSim(pivotSim, PivotConstants.pivotIn));

    // auto aim
    // turretSim.setDefaultCommand(new AutoTurretSim(turretSim));
    hoodSim.setDefaultCommand(Commands.run(() -> hoodSim.setDeg(0), hoodSim));

     DriverStation.waitForDsConnection(0);

    /* autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> Constants.isCompetition ? stream.filter(auto -> !auto.getName().startsWith("Test")): stream); */
    autoChooser.addOption("timeout left", AutoBuilder.pathfindToPose(AllianceFlipUtil.apply(DriveConstants.timeoutPoseLeft), DriveConstants.pathFindingConstraints));
    autoChooser.addOption("timeout right", AutoBuilder.pathfindToPose(AllianceFlipUtil.apply(DriveConstants.timeoutPoseRight), DriveConstants.pathFindingConstraints));
    autoChooser.addOption("salvo: shoot left", new shootAndPathToPoseSim(AllianceFlipUtil.apply(DriveConstants.ShootingStartLeft), hoodSim, turretSim, shooterSim));
    autoChooser.addOption("salvo: shoot right", new shootAndPathToPoseSim(AllianceFlipUtil.apply(DriveConstants.ShootingStartRight), hoodSim, turretSim, shooterSim));
    try{
      autoChooser.addOption("reload: shoot left | storage", new shootAndPathToPathSim(PathPlannerPath.fromPathFile("left shoot to storage"), hoodSim, turretSim, shooterSim));
      autoChooser.addOption("reload: shoot right | storage", new shootAndPathToPathSim(PathPlannerPath.fromPathFile("right shoot to storage"), hoodSim, turretSim, shooterSim));
      autoChooser.addOption("reload: shoot left | pit", new shootAndPathToPoseSim(AllianceFlipUtil.apply(DriveConstants.ShootingStartLeft), hoodSim, turretSim, shooterSim).andThen(
        new AimAndShootSim(hoodSim, turretSim, shooterSim).withTimeout(2), AutoBuilder.followPath(PathPlannerPath.fromPathFile("left shoot to pit"))));
      autoChooser.addOption("reload: shoot right | pit", new shootAndPathToPoseSim(AllianceFlipUtil.apply(DriveConstants.ShootingStartRight), hoodSim, turretSim, shooterSim).andThen(
        new AimAndShootSim(hoodSim, turretSim, shooterSim).withTimeout(2), AutoBuilder.followPath(PathPlannerPath.fromPathFile("right shoot to pit"))));
    }catch(Exception err){
      System.err.println(err);
    }
    
    SmartDashboard.putData("auto Chooser" ,autoChooser);
  }

  private void configureSimBindings(){
    // pivot toggle
    driver.a().onTrue(new PivotPidToggleSim(pivotSim));
    driver.x().onTrue(new PivotShakeSim(pivotSim));
    // shoot
    driver.b().whileTrue(new ShootSim(shooterSim));
    driver.y().whileTrue(new AimAndShootSim(hoodSim, turretSim, shooterSim));
    
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
  
  public void setupAuto(Object shooter){
   
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

// Done:
// kicker, tranfer, pivot
// intake pivot takes encoder from rollers

// kickerSim, transferSim,

// TODO: talons
//  pivotSim, intake pivot takes encoder from rollerSim
