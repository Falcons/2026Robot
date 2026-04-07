// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
// import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.Constants.IntakeConstants.RollersConstants;
import frc.robot.Constants.TurretConstants.ShooterConstants;
import frc.robot.commands.Auto.IntakeShake;
// import frc.robot.Util.AllianceFlipUtil;
// import frc.robot.commands.AimAndShootSim;
import frc.robot.commands.Auto.Setup;
import frc.robot.commands.Auto.ShootPreload;
// import frc.robot.commands.Auto.shootAndPathToPathSim;
// import frc.robot.commands.Auto.shootAndPathToPoseSim;
import frc.robot.commands.Drive.TeleopDrive;
import frc.robot.commands.Drive.taxi;
import frc.robot.commands.Hood.ManualHoodSim;
import frc.robot.commands.Intake.IntakeSim.PivotPidToggleSim;
import frc.robot.commands.Intake.IntakeSim.PivotShakeSim;
import frc.robot.commands.Turret.AutoShoot;
import frc.robot.commands.Turret.TurretPID;
import frc.robot.commands.Turret.TurretSim.ShootSim;
// import frc.robot.commands.Turret.TurretSim.ManualTurretSim; // just for sim
import frc.robot.commands.Intake.PivotPid;
// import frc.robot.commands.Intake.IntakeSim.PivotManualSim; // just for sim
import frc.robot.commands.Intake.IntakeSim.PivotPidSim;
import frc.robot.subsystems.FmsRumble;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.MiscUtils;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hood.HoodSim;
import frc.robot.subsystems.Intake.Pivot;
import frc.robot.subsystems.Intake.PivotSim;
import frc.robot.subsystems.Intake.Rollers;
import frc.robot.subsystems.Intake.RollersSim;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.subsystems.Turret.LaunchCalculator;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretSim;
import frc.robot.subsystems.Turret.Shooter.Shooter;
import frc.robot.subsystems.Turret.Shooter.ShooterSim;
import frc.robot.subsystems.Turret.Shooter.Transfer;

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
  private Lights lights;
  private Turret turret;
  private Shooter shooter;
  private Pivot pivot;
  private Hood hood;
  private Rollers rollers;
  private Transfer transfer;
  @SuppressWarnings("unused")
  private FmsRumble fmsRumble; // its being mean so i shall suppress it
  private MiscUtils miscUtils;
  
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  // private final CommandXboxController testController = new CommandXboxController(2); //TODO: remove this guy

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
    }

    // DRIVER
    driver.b().onTrue(new InstantCommand(swerve::zeroGyroWithFlip));
    // slow mode toggle
    driver.start().toggleOnTrue(Commands.startEnd(
      () -> swerve.setMaxAllowableSpeed(DriveConstants.slowModeMPS, DriveConstants.slowModeRotationsRadPS),
      () -> swerve.setMaxAllowableSpeed(DriveConstants.maxSpeedMPS, DriveConstants.maxRotationsRadPS)));
    // slow mode hold
    driver.rightBumper().whileTrue(Commands.startEnd(
      () -> swerve.setMaxAllowableSpeed(DriveConstants.slowModeMPS, DriveConstants.slowModeRotationsRadPS), 
      () -> swerve.setMaxAllowableSpeed(DriveConstants.maxSpeedMPS, DriveConstants.maxRotationsRadPS)));
    
    // add default commands (run when no other commands are running)
    swerve.setDefaultCommand(new TeleopDrive( 
      swerve, 
      () -> MathUtil.applyDeadband(-driver.getLeftX(), ControllerConstants.deadBand), 
      () -> MathUtil.applyDeadband(-driver.getLeftY(), ControllerConstants.deadBand),
      () -> MathUtil.applyDeadband(-driver.getRightX(), ControllerConstants.deadBand), 
      () -> !driver.getHID().getLeftBumper()));
  }
  
  private void setupReal() {
    // DriverStation.silenceJoystickConnectionWarning(true);
    //initializing real classes
    this.lights = new Lights();
    this.turret = new Turret(swerve, lights);
    this.transfer = new Transfer(turret, lights);
    this.shooter = new Shooter(turret, swerve, lights);
    this.transfer.setShooter(shooter);
    this.rollers = new Rollers(lights);
    this.pivot = new Pivot(swerve);
    this.hood = new Hood(swerve, lights);
    this.fmsRumble = new FmsRumble(new CommandXboxController[]{driver, operator});
    

    SmartDashboard.putNumber("Turret/Shooter/Fire speed Rps", 60);
    SmartDashboard.putNumber("Intake/Rollers/Set RPM", 3000);
    SmartDashboard.putBoolean("Intake/Rollers/rollers active", !Constants.isCompetition);
    SmartDashboard.putBoolean("Turret/Shooter/driver shoot active", !Constants.isCompetition);

    // preloadFire = new ShootPreload(turret, hood, transfer, shooter, rollers, pivot);
    // Named Commands
    NamedCommands.registerCommand("Shoot", new AutoShoot(turret, hood, transfer, shooter, rollers).withTimeout(10));
    NamedCommands.registerCommand("Intake out", new PivotPid(pivot, PivotConstants.pivotOut).withTimeout(1));
    NamedCommands.registerCommand("Intake in", new PivotPid(pivot, PivotConstants.pivotIn).withTimeout(1));
    NamedCommands.registerCommand("Delayed intake shake", new IntakeShake(pivot, 1));
    NamedCommands.registerCommand("Intake shake", new IntakeShake(pivot, 0));
    NamedCommands.registerCommand("Rollers", Commands.runEnd(() -> rollers.setRPS(RollersConstants.rollerSpeedRPS), rollers::stop, rollers));
    NamedCommands.registerCommand("Rollers 1 sec", Commands.runEnd(() -> rollers.setRPS(RollersConstants.rollerSpeedRPS), rollers::stop, rollers).withTimeout(1));
    // NamedCommands.registerCommand("inverted gyro zero", Commands.runOnce(swerve::invertedZeroGyroWithFlip));
    //autos
    DriverStation.waitForDsConnection(0);

    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> Constants.isCompetition
        ? stream.filter(auto -> !auto.getName().startsWith("Test")): stream);
    
    autoChooser.setDefaultOption("setup", new Setup(pivot, swerve));

    SmartDashboard.putBoolean("shoot preload", true);
    SmartDashboard.putData("auto Chooser" ,autoChooser);

    transfer.setDefaultCommand(Commands.run(() -> transfer.set(
        () -> operator.getRightTriggerAxis() > ControllerConstants.triggerDeadBand, ShooterConstants.maxTransferSpeed), transfer));

    // pivot.setDefaultCommand(Commands.runEnd(() -> pivot.set(operator::getRightY), () -> pivot.set(0), pivot));
    // this.miscUtils = new MiscUtils(autoChooser, swerve);
  }

  private void configureRealBindings() {
    
    driver.povUp().onTrue(new InstantCommand(() -> LaunchCalculator.setSpeedOffset(LaunchCalculator.getSpeedOffset() + 0.05)));
    driver.povDown().onTrue(new InstantCommand(() -> LaunchCalculator.setSpeedOffset(LaunchCalculator.getSpeedOffset() - 0.05)));
    driver.povLeft().onTrue(new InstantCommand(() -> LaunchCalculator.setSpeedOffset(0)));

    driver.povRight().whileTrue(Commands.runEnd(() -> rollers.setRPS(() -> SmartDashboard.getNumber("Intake/Rollers/Set RPM", 3000)), rollers::stop, rollers));

    // intake
    driver.rightTrigger().whileTrue(Commands.runEnd(() -> rollers.setRPS(
      SmartDashboard.getBoolean("Intake/Rollers/rollers active", !Constants.isCompetition) ? RollersConstants.rollerSpeedRPS : 0), 
      rollers::stop, rollers));
    // shoot
    driver.leftTrigger().whileTrue(
      new AutoShoot(turret, hood, transfer, shooter, rollers).onlyWhile(
        () -> SmartDashboard.getBoolean("Turret/Shooter/driver shoot active", !Constants.isCompetition)));

    // OPERATOR

    // spin shooter
    operator.back().whileTrue(Commands.runEnd(() -> shooter.setShooter(1.0), shooter::stopShooter, shooter));  
    operator.axisMagnitudeGreaterThan(2, ControllerConstants.triggerDeadBand).whileTrue(Commands.runEnd(
      () -> shooter.setRps(() -> SmartDashboard.getNumber("Turret/Shooter/Fire speed Rps", 60)), 
      shooter::stopShooter, shooter));
    operator.leftBumper().whileTrue(Commands.runEnd(() -> shooter.setRps(85), shooter::stopShooter, shooter));

    // manual turret 50% speed
    operator.axisMagnitudeGreaterThan(0, ControllerConstants.deadBand).whileTrue( 
      Commands.runEnd(() -> turret.set(() -> operator.getLeftX() * 0.5), () -> turret.set(0), turret));
    // manual pivot
    // operator.axisMagnitudeGreaterThan(0, ControllerConstants.deadBand).whileTrue(
    //   Commands.runEnd(() -> pivot.set(() -> operator.getLeftX()), () -> pivot.set(0), pivot));

    // auto turret only
    operator.b().whileTrue(Commands.runEnd(turret::autoLimelightAim, turret::stop, turret));
    operator.a().whileTrue(new AutoShoot(turret, hood, transfer, shooter, rollers).andThen(new PrintCommand("autoShoot ended")));

    // hood
    operator.axisMagnitudeGreaterThan(5, 0.95).onTrue(
      new InstantCommand(() -> hood.moveHood(operator::getRightY), hood));

    // spin intake - rollers 
    // operator.x().whileTrue(Commands.runEnd(() -> rollers.set(RollersConstants.rollerSpeed), rollers::stop, rollers));
    // operator.rightBumper().whileTrue(Commands.runEnd(() -> rollers.set(RollersConstants.slowRollerSpeed), rollers::stop, rollers));
    operator.x().whileTrue(Commands.runEnd(() -> rollers.setRPS(RollersConstants.rollerSpeedRPS), rollers::stop, rollers));
    operator.rightBumper().whileTrue(Commands.runEnd(() -> rollers.setRPS(RollersConstants.slowRollerSpeedRPS), rollers::stop, rollers));
    operator.povRight().whileTrue(Commands.runEnd(() -> rollers.set(-RollersConstants.rollerSpeed), () -> rollers.set(0), rollers));

    // intake out and in
    operator.povUp().onTrue(new PivotPid(pivot, PivotConstants.pivotOut));
    operator.povDown().onTrue(new PivotPid(pivot, PivotConstants.pivotIn));
    // shake
    // operator.povLeft().onTrue(new PivotPid(pivot, PivotConstants.pivotIn));
    operator.povLeft().onTrue(new PivotPid(pivot, PivotConstants.pivotIn)); 

    operator.povLeft().onFalse(new PivotPid(pivot, PivotConstants.pivotOut));

    // return home
    operator.y().whileTrue(
      new ParallelCommandGroup(
      new TurretPID(turret, Math.toRadians(90)),
      new InstantCommand(() -> hood.set(HoodConstants.hoodMin))));
    
    // driver.y().whileTrue(Commands.runEnd(() -> shooter.playSong("src/main/deploy/chirp/crazy_train.chrp"), shooter::stopSong)); //music bs

    // SYS ID
    // testController.a().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // testController.b().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // testController.x().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // testController.y().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // TODO: test rps
    //operator.start().whileTrue(Commands.runEnd(shooter::shooterTest, shooter::stopShooter, shooter));
  }
 
  private void setupSim(){
    // initializing sim  classes

    // need to do null shenangigans since they all depend on each other ._.
    this.turretSim = new TurretSim(swerve); 
    this.shooterSim = new ShooterSim(turretSim);

    this.hoodSim = new HoodSim(swerve);
    this.rollersSim = new RollersSim();
    this.pivotSim = new PivotSim(rollersSim);

    // silence joystick warning
    DriverStation.silenceJoystickConnectionWarning(true);

    // Named Commands
    NamedCommands.registerCommand("Test", new PrintCommand("test"));
    // NamedCommands.registerCommand("AimAndShoot", new AimAndShootSim(hoodSim, turretSim, shooterSim));
    NamedCommands.registerCommand("Shake", new PivotShakeSim(pivotSim));
    NamedCommands.registerCommand("Intake", new PivotPidSim(pivotSim, PivotConstants.pivotOut));
    NamedCommands.registerCommand("Outake", new PivotPidSim(pivotSim, PivotConstants.pivotIn));

    // pathplaner events
    new EventTrigger("Intake out").onTrue(new PivotPidSim(pivotSim, PivotConstants.pivotOut));
    new EventTrigger("Inatke in").onTrue(new PivotPidSim(pivotSim, PivotConstants.pivotIn));

    // auto aim
    // turretSim.setDefaultCommand(new AutoTurretSim(turretSim));
    hoodSim.setDefaultCommand(Commands.run(() -> hoodSim.setDeg(0), hoodSim));

     DriverStation.waitForDsConnection(0);

    /* autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> Constants.isCompetition ? stream.filter(auto -> !auto.getName().startsWith("Test")): stream); */
    // autoChooser.addOption("timeout left", AutoBuilder.pathfindToPose(AllianceFlipUtil.apply(DriveConstants.timeoutPoseLeft), DriveConstants.pathFindingConstraints));
    // autoChooser.addOption("timeout right", AutoBuilder.pathfindToPose(AllianceFlipUtil.apply(DriveConstants.timeoutPoseRight), DriveConstants.pathFindingConstraints));
    // autoChooser.addOption("salvo: shoot left", new shootAndPathToPoseSim(AllianceFlipUtil.apply(DriveConstants.ShootingStartLeft), hoodSim, turretSim, shooterSim));
    // autoChooser.addOption("salvo: shoot right", new shootAndPathToPoseSim(AllianceFlipUtil.apply(DriveConstants.ShootingStartRight), hoodSim, turretSim, shooterSim));
    // try{
    //   autoChooser.addOption("reload: shoot left | storage", new shootAndPathToPathSim(PathPlannerPath.fromPathFile("left shoot to storage"), hoodSim, turretSim, shooterSim));
    //   autoChooser.addOption("reload: shoot right | storage", new shootAndPathToPathSim(PathPlannerPath.fromPathFile("right shoot to storage"), hoodSim, turretSim, shooterSim));
    //   autoChooser.addOption("reload: shoot left | pit", new shootAndPathToPoseSim(AllianceFlipUtil.apply(DriveConstants.ShootingStartLeft), hoodSim, turretSim, shooterSim).andThen(
    //     new AimAndShootSim(hoodSim, turretSim, shooterSim).withTimeout(2), AutoBuilder.followPath(PathPlannerPath.fromPathFile("left shoot to pit"))));
    //   autoChooser.addOption("reload: shoot right | pit", new shootAndPathToPoseSim(AllianceFlipUtil.apply(DriveConstants.ShootingStartRight), hoodSim, turretSim, shooterSim).andThen(
    //     new AimAndShootSim(hoodSim, turretSim, shooterSim).withTimeout(2), AutoBuilder.followPath(PathPlannerPath.fromPathFile("right shoot to pit"))));
    // }catch(Exception err){
    //   System.err.println(err);
    // }
    
    
    SmartDashboard.putData("auto Chooser" ,autoChooser);
  }
  private void configureSimBindings(){
    // pivot toggle
    driver.a().onTrue(new PivotPidToggleSim(pivotSim));
    driver.x().onTrue(new PivotShakeSim(pivotSim));
    // shoot
    driver.b().whileTrue(new ShootSim(shooterSim));
    // driver.y().whileTrue(new AimAndShootSim(hoodSim, turretSim, shooterSim));
    
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
      Command currentAuto = autoChooser.getSelected();
      CommandScheduler.getInstance().removeComposedCommand(currentAuto);

      System.out.print("Selected auto: " + currentAuto.getName());
      if(SmartDashboard.getBoolean("shoot preload", true)){
        System.out.print(" + shooting preload");
        return new SequentialCommandGroup(
          new PivotPid(pivot, PivotConstants.pivotOut).withTimeout(1),
          new ParallelCommandGroup(
            new AutoShoot(turret, hood, transfer, shooter, rollers).withTimeout(5),
            new SequentialCommandGroup(
              new WaitCommand(1),
              new PivotPid(pivot, PivotConstants.pivotIn).withTimeout(1),
              new WaitCommand(1),
              new PivotPid(pivot, PivotConstants.pivotOut).withTimeout(1)
            )
          ),
          currentAuto);
      }
      System.out.println();
      // return currentAuto;
      return currentAuto;
    }catch (Exception err){
      System.err.println("error loading autonomous command | " + err);
      return new taxi(swerve, 1);
    }
  }
}