// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import frc.robot.Constants.IntakeConstants.PivotConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
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

  PIDController pivotPid = new PIDController(0.3, 0, 0);

  // Simulated motor
  private final SparkMax pivotMotor = new SparkMax(Math.abs(PivotConstants.pivotCANID-200), MotorType.kBrushless);
  private final SparkMaxSim pivotSim = new SparkMaxSim(pivotMotor, DCMotor.getNEO(1));

  // Motor Encoder
  private final SparkAbsoluteEncoderSim pivotEncoderSim = pivotSim.getAbsoluteEncoderSim();

  private boolean atMax = false, atMin = false;

  // Simulated variables
  private Rotation2d pivotDir = new Rotation2d();
  private Pose2d pivotPose = new Pose2d();

  /** Creates a new PivotSim. */
  public PivotSim() {
    SmartDashboard.putData("Field", field);
    // pivotPid.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    atMax = pivotEncoderSim.getPosition() >= PivotConstants.pivotIn;
    atMin = pivotEncoderSim.getPosition() <= PivotConstants.pivotOut;
    
    pivotDir = new Rotation2d(pivotEncoderSim.getPosition());
    pivotPose = new Pose2d(new Translation2d(0, 0), pivotDir);
    field.getObject("IntakePivot").setPose(pivotPose);

    SmartDashboard.putNumber("Intake/PivotSim/Angle", pivotDir.getDegrees());
  }

  public void setPivot(double speed) { 
    // clamp
    if (atMax && speed > 0) {
      speed = 0; pivotMotor.stopMotor();
    } 
    if (atMin && speed < 0) {
      speed = 0; pivotMotor.stopMotor();
    }
    pivotSim.setAppliedOutput(speed);
    pivotEncoderSim.setPosition(pivotEncoderSim.getPosition() + speed);
    SmartDashboard.putNumber("Intake/PivotSim/Speed", speed);
  }

  /**
   * @param setpoint the radians to set the pivot
   */
  public void setPivotPid(double setpoint) {
    setpoint = MathUtil.clamp(setpoint, PivotConstants.pivotOut, PivotConstants.pivotIn);
    double pid = pivotPid.calculate(pivotEncoderSim.getPosition(), setpoint);
    // check if above setpoint clamp, and clamp pid speed
    pid = MathUtil.clamp(pid, -0.5, 0.5);

    SmartDashboard.putNumber("Intake/PivotSim/PID/setpoint", setpoint);
    SmartDashboard.putNumber("Intake/PivotSim/PID/calc", pid);
    setPivot(pid);
    // pivotPid.reset();
  }
  public double getPivotDegrees() {
    return Math.toDegrees(pivotEncoderSim.getPosition());
  }

  public boolean atSetpoint() {
    return pivotPid.atSetpoint();
  }
}
