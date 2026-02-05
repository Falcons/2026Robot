// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import frc.robot.Constants.IntakeConstants.PivotConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSim extends SubsystemBase {

  private final Field2d field = new Field2d();

  // Simulated motor
  private final SparkMax pivotMotor = new SparkMax(PivotConstants.pivotCANID, MotorType.kBrushless);
  private final SparkMaxSim pivotSim = new SparkMaxSim(pivotMotor, DCMotor.getNEO(1));

  // Motor Encoder
  private final SparkAbsoluteEncoderSim pivotEncoderSim = pivotSim.getAbsoluteEncoderSim();

  private final PIDController pivotPID = new PIDController(0.05, 0, 0);

  private boolean atMax = false, atMin = false;

  // Simulated variables
  private Rotation2d pivotDir = new Rotation2d();
  private Pose2d pivotPose = new Pose2d();

  /** Creates a new PivotSim. */
  public PivotSim() {
    SmartDashboard.putData("Field", field);
    pivotPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pivotDir = new Rotation2d(pivotEncoderSim.getPosition());
    pivotPose = new Pose2d(new Translation2d(0, 0), pivotDir);
    field.getObject("IntakePivot").setPose(pivotPose);

    SmartDashboard.putNumber("Pivot/MovementSim/Angle", pivotDir.getDegrees());
  }
}
