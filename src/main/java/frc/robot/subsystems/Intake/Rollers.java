// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.RollersConstants;

public class Rollers extends SubsystemBase {
  private final SparkMax roller;
  private SparkMaxConfig rollerConfig = new SparkMaxConfig();
  /** Creates a new Rollers. */
  public Rollers() {
    this.roller = new SparkMax(RollersConstants.rollerCANID, MotorType.kBrushless);
    rollerConfig.smartCurrentLimit(20);
    roller.configure(rollerConfig, com.revrobotics.ResetMode.kResetSafeParameters,  com.revrobotics.PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Roller/Encoder", roller.getEncoder().getVelocity());
  }

  public void set(double roll) {
    SmartDashboard.putNumber("Intake/Roller/Speed", roll);
    roller.set(roll);
  }

  public void stop() {
    roller.stopMotor();
  }

  public double getRollerVelocity() {
    return roller.getEncoder().getVelocity()/3.0;
  }
}
