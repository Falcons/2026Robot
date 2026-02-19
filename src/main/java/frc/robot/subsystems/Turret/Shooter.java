// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import com.revrobotics.spark.SparkMax;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants.ShooterConstants;

public class Shooter extends SubsystemBase {
  
  // once shooters AND transfer are max speed than kicker
  private final TalonFX leftShooter = new TalonFX(ShooterConstants.leftShooterCANID);
  private final TalonFX rightShooter = new TalonFX(ShooterConstants.rightShooterCANID);
  private final SparkMax kicker = new SparkMax(ShooterConstants.kickerCANID, MotorType.kBrushless);

  private final SparkMax transfer = new SparkMax(ShooterConstants.transferCANID, MotorType.kBrushless);

  // configs
  private final TalonFXConfiguration leftShooterConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration rightShooterConfig = new TalonFXConfiguration();
  private final SparkMaxConfig kickerConfig = new SparkMaxConfig();

  private final SparkMaxConfig transferConfig = new SparkMaxConfig();

  // need to connect to movement to see if its aligned
  private final Turret aimer;

  // variables
  public boolean shooterRunning = false;

  /** Creates a new Shooter. */
  public Shooter(Turret aimer) {

    this.aimer = aimer;

    // follow right shooter
    leftShooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftShooter.setControl(new Follower(ShooterConstants.rightShooterCANID, MotorAlignmentValue.Opposed));

    // apply configs
    leftShooter.getConfigurator().apply(leftShooterConfig);
    rightShooter.getConfigurator().apply(rightShooterConfig);
    transfer.configure(transferConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kicker.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * turns on transfer motor when the shooter reaches max speed
   */
  public void shootWhenMaxSpeed() {
    // dont run if not in range
    if (!aimer.inRange()) return;

    rightShooter.set(ShooterConstants.maxShooterSpeed);
    kicker.set(ShooterConstants.maxKickerSpeed);
    // wait until shooter is max speed than rotate transfer
    if (rightShooter.getVelocity().getValueAsDouble() >= ShooterConstants.maxShooterRPS) {
      transfer.set(ShooterConstants.maxTransferSpeed);
      shooterRunning = true;
    }
  }

  public void pulseTransfer() {}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret/Shooter/leftShooterSpeed", leftShooter.get());
    SmartDashboard.putNumber("Turret/Shooter/rightShooterSpeed", rightShooter.get());
    SmartDashboard.putNumber("Turret/Shooter/transferSpeed", transfer.get());
    SmartDashboard.putNumber("Turret/Shooter/kickerSpeed", kicker.get());

    SmartDashboard.putNumber("Turret/Shooter/rightShooterVelocity", rightShooter.getVelocity().getValueAsDouble());
  }

  /**
   * check if the shooter rpm is too low to shoot
   * @return true when rpm is too low
   */
  public boolean shooterRPMlow() {
    if (shooterRunning && rightShooter.getVelocity().getValueAsDouble() < ShooterConstants.maxShooterRPS) {
      shooterRunning = false;
      return true;
    }
    return false;
  }

  /**
   * set the transfer speed
   * @param speed the speed to set
   */
  public void setTransfer(double speed) {
    transfer.set(speed);
  }
  /**
   * set the kicker speed
   * @param speed the speed to set
   */
  public void setKicker(double speed) {
    kicker.set(speed);
  }
  /**
   * set the shooter speed
   * @param speed the speed to set
   */
  public void setShooter(DoubleSupplier speed) {
    rightShooter.set(speed.getAsDouble());
  }
  /**
   * set the shooter and transfer speed
   * @param transferSpeed transfer speed
   * @param shooterSpeed shooter speed
   * @param kickerSpeed kciker speed
   */
  public void fullShoot(double transferSpeed, double kickerSpeed, DoubleSupplier shooterSpeed) {
    rightShooter.set(shooterSpeed.getAsDouble());
    transfer.set(transferSpeed);
    kicker.set(kickerSpeed);
  }
  /**
   * set the shooter and kicker speed
   * @param shooterSpeed shooter speed
   * @param kickerSpeed kciker speed
   */
  public void setShooterWithkicker(DoubleSupplier shooterSpeed, double kickerSpeed) {
    rightShooter.set(shooterSpeed.getAsDouble());
    kicker.set(kickerSpeed);
  }
}
