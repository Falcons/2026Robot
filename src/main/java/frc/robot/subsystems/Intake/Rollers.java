// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.RollersConstants;

public class Rollers extends SubsystemBase {
  private final SparkMax roller = new SparkMax(RollersConstants.rollerCANID, MotorType.kBrushless);
  private final AbsoluteEncoder pivotEncoder = roller.getAbsoluteEncoder();
  private SparkMaxConfig rollerConfig = new SparkMaxConfig();
  /** Creates a new Rollers. */
  public Rollers() {
    rollerConfig.encoder.positionConversionFactor(Math.PI / 180); // convert from deg to rad

    rollerConfig.smartCurrentLimit(20);
    roller.configure(rollerConfig, com.revrobotics.ResetMode.kResetSafeParameters,  com.revrobotics.PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
  }

  public void set(double roll) {
    roller.set(roll);
  }

  public void stop() {
    roller.stopMotor();
  }

  public AbsoluteEncoder getPivotEncoder() {
    return pivotEncoder;
  }
}
