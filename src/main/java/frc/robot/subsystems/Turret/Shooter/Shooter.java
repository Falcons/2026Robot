// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret.Shooter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants.ShooterConstants;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.subsystems.Turret.LaunchCalculator;
import frc.robot.subsystems.Turret.Turret;

public class Shooter extends SubsystemBase {
  
  // Orchestra orchestra = new Orchestra();///music bs

  // once shooters AND kicker are max speed than transfer
  private final TalonFX leftShooter = new TalonFX(ShooterConstants.leftShooterCANID); // Kraken x60
  private final TalonFX rightShooter = new TalonFX(ShooterConstants.rightShooterCANID);
  private final TalonFX kicker = new TalonFX(ShooterConstants.kickerCANID);

  // configs
  private final TalonFXConfiguration leftShooterConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration rightShooterConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration kickerConfig = new TalonFXConfiguration();

  private final CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs();


  // need to connect to movement to see if its aligned
  private final Turret aimer;
  private final Swerve swerve;

  // variables
  public boolean shooterRunning = false;
  private double shooterSetSpeed = 0;
  private double shooterAutoSpeed = 0;

  private final PIDController speedControl = new PIDController(0.46, 0, 0.05);

  /** Creates a new Shooter. */
  public Shooter(Turret aimer, Swerve swerve) {
    this.aimer = aimer;
    this.swerve = swerve;

    // smart current limits
    limitsConfigs.StatorCurrentLimit = 40;
    limitsConfigs.StatorCurrentLimitEnable = true;

    // coast
    leftShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // follow right shooter
    leftShooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightShooter.setControl(new Follower(ShooterConstants.leftShooterCANID, MotorAlignmentValue.Opposed));
    // kicker.setControl(new Follower(ShooterConstants.leftShooterCANID, MotorAlignmentValue.Aligned));

    // apply configs
    leftShooterConfig.withCurrentLimits(limitsConfigs);
    leftShooter.getConfigurator().apply(leftShooterConfig);
    
    rightShooterConfig.withCurrentLimits(limitsConfigs);
    rightShooter.getConfigurator().apply(rightShooterConfig);
    kicker.getConfigurator().apply(kickerConfig);

    //  music bs  
    // orchestra.addInstrument(leftShooter);
    // orchestra.addInstrument(rightShooter);
  }

  /* 
  public void playSong(String path){
    orchestra.loadMusic(path);
    orchestra.play();
  }
  public void stopSong(){
    orchestra.stop();
  }*/

  /**
   * shoot based on what launch calc spits out
   */
  public void autoShoot() {
    if (!aimer.inRange()) return; // dont shoot if not in range

    LaunchCalculator.getInstance().clearLaunchingParameters();
    shooterAutoSpeed = LaunchCalculator.getInstance().getParameters(swerve, -1.0).flywheelSpeed();
    this.setRps(shooterAutoSpeed);
  }

  public double getShooterAutoSpeed() {
    return shooterAutoSpeed;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret/Shooter/leftShooterSpeed", leftShooter.get());
    SmartDashboard.putNumber("Turret/Shooter/rightShooterSpeed", rightShooter.get());
    SmartDashboard.putNumber("Turret/Shooter/leftShooterRPS", leftShooter.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Turret/Shooter/rightShooterRPS", rightShooter.getVelocity().getValueAsDouble());
    
    SmartDashboard.putNumber("Turret/Shooter/kickerSpeed", kicker.get());
    SmartDashboard.putBoolean("Turret/Shooter/shooterRunning", shooterRunning);

    SmartDashboard.putNumber("Turret/Shooter/PID/setpoint", speedControl.getSetpoint());
    SmartDashboard.putNumber("Turret/Shooter/PID/error", speedControl.getError());

    SmartDashboard.putNumber("Turret/Shooter/LaunchCalc/Speed offset", LaunchCalculator.getSpeedOffset());
  }

  /**
   * set the shooter and kicker speed
   * @param speed the speed to set
   */
  public void setShooter(DoubleSupplier speed) {
    leftShooter.set(speed.getAsDouble());
    shooterSetSpeed = speed.getAsDouble();
  }

  /**
   * set the shooter and kicker speed
   * @param speed the speed to set
   */
  public void setShooter(Double speed) {
    leftShooter.set(speed);
    shooterSetSpeed = speed;
  }

  public void setRps(double speed){
    double pid = speedControl.calculate(getShooterRPS(), speed);
    SmartDashboard.putNumber("Turret/Shooter/PID/raw pid", pid);
    pid /= 97; // max rps
    SmartDashboard.putNumber("Turret/Shooter/PID/adjusted pid", pid);
    setShooter(leftShooter.get() + pid);
    kicker.set(1);
  }

  public void setRps(DoubleSupplier speed){
    setRps(speed.getAsDouble());
  }
  public void stopShooter(){
    leftShooter.stopMotor();
    rightShooter.stopMotor();
    kicker.stopMotor();
  }
  /**
   * get the rps of the shooter
   */
  public Double getShooterRPS() {
    return leftShooter.getVelocity().getValueAsDouble();
  }

  public Double getShooterRealSpeed(){
    return leftShooter.get();
  }

  public Double getShooterSetSpeed(){
    return shooterSetSpeed;
  }

  public double getShooterRealRPS() {
    return leftShooter.getVelocity().getValueAsDouble();
  }
}
