// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.RollersConstants;

public class RollersSim extends SubsystemBase {
  private final SparkMax roller = new SparkMax(RollersConstants.rollerCANID, MotorType.kBrushless); 
  private final SparkMaxSim rollerSim = new SparkMaxSim(roller, DCMotor.getNEO(1));

  /** Creates a new Rollers. */
  public RollersSim() {}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Roller velocity", rollerSim.getVelocity());
  }

  /**
   * set rollers
   */
  public void set(double roll) {
    rollerSim.setVelocity(roll);
  }

  /**
   * get encoder for the pivot, YES FOR THE PIVOT
   */
  public SparkAbsoluteEncoderSim getPivotEncoder() {
    return rollerSim.getAbsoluteEncoderSim();
  }
}
