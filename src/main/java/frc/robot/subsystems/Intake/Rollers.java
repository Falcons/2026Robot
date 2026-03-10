// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.RollersConstants;

public class Rollers extends SubsystemBase {
  private final SparkMax roller;
  private SparkMaxConfig rollerConfig = new SparkMaxConfig();
  /** Creates a new Rollers. */
  public Rollers() {
    this.roller = new SparkMax(RollersConstants.rollerCANID, MotorType.kBrushless); 
    rollerConfig.idleMode(IdleMode.kCoast);
    rollerConfig.smartCurrentLimit(20);
    rollerConfig.absoluteEncoder.positionConversionFactor(360 * Math.PI / 180); // deg to rad is pi / 180
    roller.configure(rollerConfig, com.revrobotics.ResetMode.kResetSafeParameters,  com.revrobotics.PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Rollers/Roller speed", roller.get());
  }

  /**
   * set rollers
   */
  public void set(double roll) {
    roller.set(roll);
  }

  /**
   * stop rollers
   */
  public void stop() {
    roller.stopMotor();
  }

  /**
   * get encoder for the pivot, YES FOR THE PIVOT
   */
  public AbsoluteEncoder getPivotEncoder() {
    return roller.getAbsoluteEncoder();
  }
}
