// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.Drive.TeleopDrive;
import frc.robot.commands.Drive.ZeroGyro;
import frc.robot.commands.Intake.IntakeSim.PivotPidToggleSim;
import frc.robot.commands.Intake.IntakeSim.PivotShakeSim;
import frc.robot.commands.Turret.TurretSim.AutoTurretSim;
import frc.robot.commands.Turret.TurretSim.ManualTurretSim; // DONT REMOVE
import frc.robot.commands.Intake.IntakeSim.PivotManualSim;
import frc.robot.subsystems.Intake.PivotSim;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.subsystems.Turret.TurretSim.MovementSim;

public class RobotContainer {

  // initialize robot's subsystems
  private final Swerve swerve = new Swerve();

  // simulated classes
  private final MovementSim movementSim = new MovementSim(swerve);
  private final PivotSim pivotSim = new PivotSim();
  
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
  
    
    // Configure the button bindings
    configureBindings();

    // SIM CONTROLS:
    if (RobotBase.isReal()){return;}

    // auto aim
    movementSim.setDefaultCommand(new AutoTurretSim(movementSim));
  }
  
  private void configureBindings() {
    

    driver.b().onTrue(new ZeroGyro(swerve));

    // SIM CONTROLS:
    if (RobotBase.isReal()){return;}

    // pivot toggle
    driver.a().onTrue(new PivotPidToggleSim(pivotSim));
    driver.x().onTrue(new PivotShakeSim(pivotSim));

    // manual turret
    // driver.axisMagnitudeGreaterThan(4, ControllerConstants.deadBand).whileTrue(
    //   new ManualTurretSim(
    //   movementSim, 
    //   () -> -driver.getRightX()));
    // manual pivot
    driver.axisMagnitudeGreaterThan(5, ControllerConstants.deadBand).whileTrue(
      new PivotManualSim(
      pivotSim, 
      () -> driver.getRightY()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}