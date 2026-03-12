// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret.Shooter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.Orchestra;
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
  private final Transfer transfer;
  private final Swerve swerve;

  // variables
  public boolean shooterRunning = false;
  private double shooterSetSpeed = 0;
  private double shooterAutoSpeed = 0;

  private final PIDController speedControl = new PIDController(5, 0, 0);

  /** Creates a new Shooter. */
  public Shooter(Turret aimer, Transfer transfer, Swerve swerve) {
    this.aimer = aimer;
    this.transfer = transfer;
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
    kicker.setControl(new Follower(ShooterConstants.leftShooterCANID, MotorAlignmentValue.Aligned));

    // apply configs
    leftShooterConfig.withCurrentLimits(limitsConfigs);
    leftShooter.getConfigurator().apply(leftShooterConfig);
    
    rightShooterConfig.withCurrentLimits(limitsConfigs);
    rightShooter.getConfigurator().apply(rightShooterConfig);
    kicker.getConfigurator().apply(kickerConfig);

    /* music bs  
    orchestra.addInstrument(leftShooter);
    orchestra.addInstrument(rightShooter);
    orchestra.addInstrument(transfer); */
  }

  /*
  public void playSong(String path){
    orchestra.loadMusic(path);
    orchestra.play();
  }
  public void stopSong(){
    orchestra.stop();
  }
    */

  /**
   * turns on transfer motor when the shooter reaches max speed
   */
  public void shootWhenMaxSpeed() {
    // dont run if not in range
    if (!aimer.inRange()) return;

    leftShooter.set(ShooterConstants.maxShooterSpeed);
    // wait until shooter is max speed than rotate transfer
    if (leftShooter.getVelocity().getValueAsDouble() >= ShooterConstants.maxShooterRPS) {
      transfer.pulse();
      shooterRunning = true;
    }
  }

  /**
   * shoot based on what launch calc spits out
   */
  public void autoShoot() {
    LaunchCalculator.getInstance().clearLaunchingParameters();
    shooterAutoSpeed = LaunchCalculator.getInstance().getParameters(swerve, -1.0).flywheelSpeed();
    leftShooter.set(shooterAutoSpeed);
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
  }

  /**
   * check if the shooter rps is too low to shoot
   * @return true when rps is too low
   */
  public boolean shooterRPSlow() {
    if (shooterRunning && leftShooter.getVelocity().getValueAsDouble() < ShooterConstants.maxShooterRPS) {
      shooterRunning = false;
      return true;
    }
    return false;
  }

  /**
   * set the kicker speed
   * @param speed the speed to set
   */
  public void setKicker(double speed) {
    kicker.set(speed);
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
  public void setRps(double speed){ //TODO: get numbers
    double pid = speedControl.calculate(getShooterRPS(), speed);
    pid /= 113.07; // max rps
    setShooter(pid);
  }
  public void setRps(DoubleSupplier speed){ //TODO: get numbers
    setRps(speed.getAsDouble());
  }
  public void stopShooter(){
    leftShooter.stopMotor();
    rightShooter.stopMotor();
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
}
