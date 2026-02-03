// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants.ShooterConstants;

public class Shooter extends SubsystemBase {
  
  // once shooters AND transfer are max speed than kicker
  private final SparkMax leftShooter = new SparkMax(ShooterConstants.leftShooter, MotorType.kBrushless);
  private final SparkMax rightShooter = new SparkMax(ShooterConstants.rightShooter, MotorType.kBrushless);
  private final SparkMax transfer = new SparkMax(ShooterConstants.transfer, MotorType.kBrushless);

  private final SparkMax kicker = new SparkMax(ShooterConstants.kicker, MotorType.kBrushless);

  // configs
  private final SparkMaxConfig leftShooterConfig = new SparkMaxConfig();
  private final SparkMaxConfig rightShooterConfig = new SparkMaxConfig();
  private final SparkMaxConfig transferConfig = new SparkMaxConfig();

  private final SparkMaxConfig kickerConfig = new SparkMaxConfig();

  /** Creates a new Shooter. */
  public Shooter() {

    // apply configs
    leftShooter.configure(leftShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightShooter.configure(rightShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    transfer.configure(transferConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kicker.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
