// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants.ShooterConstants;

public class Shooter extends SubsystemBase {
  
  // once shooters AND transfer are max speed than kicker
  private final SparkMax leftShooter = new SparkMax(ShooterConstants.leftShooterCANID, MotorType.kBrushless);
  private final SparkMax rightShooter = new SparkMax(ShooterConstants.rightShooterCANID, MotorType.kBrushless);
  private final SparkMax kicker = new SparkMax(ShooterConstants.kickerCANID, MotorType.kBrushless);

  private final SparkMax transfer = new SparkMax(ShooterConstants.transferCANID, MotorType.kBrushless);

  // configs
  private final SparkMaxConfig leftShooterConfig = new SparkMaxConfig();
  private final SparkMaxConfig rightShooterConfig = new SparkMaxConfig();
  private final SparkMaxConfig kickerConfig = new SparkMaxConfig();

  private final SparkMaxConfig transferConfig = new SparkMaxConfig();

  // need to connect to movement to see if its aligned
  private final Movement aimer;

  /** Creates a new Shooter. */
  public Shooter(Movement aimer) {

    this.aimer = aimer;

    // follow right shooter
    leftShooterConfig
      .inverted(true) // left motor is inverted
      .follow(ShooterConstants.rightShooterCANID);
    kickerConfig.follow(ShooterConstants.rightShooterCANID);

    // apply configs
    leftShooter.configure(leftShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightShooter.configure(rightShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    transfer.configure(transferConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kicker.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * turns on transfer motor when the shooter reaches max speed
   */
  public void shootWhenMaxSpeed() {
    // dont run if not in range
    if (!aimer.inRange()){return;}

    rightShooter.set(ShooterConstants.maxShooterSpeed);
    // wait until shooter is max speed than rotate transfer
    if (rightShooter.getEncoder().getVelocity() >= ShooterConstants.maxShooterSpeed) {
      transfer.set(ShooterConstants.maxShooterSpeed);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret/Shooter/leftShooterSpeed", leftShooter.get());
    SmartDashboard.putNumber("Turret/Shooter/rightShooterSpeed", rightShooter.get());
    SmartDashboard.putNumber("Turret/Shooter/transferSpeed", transfer.get());
    SmartDashboard.putNumber("Turret/Shooter/kickerSpeed", kicker.get());

    SmartDashboard.putNumber("Turret/Shooter/rightShooterVelocity", rightShooter.getEncoder().getVelocity());
  }
}
