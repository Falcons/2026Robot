// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.Constants.IntakeConstants.RollersConstants;

public class Pivot extends SubsystemBase {
  // pivot motor, encoder and config
  private final SparkMax pivot = new SparkMax(PivotConstants.pivotCANID, MotorType.kBrushless);
  private final SparkAbsoluteEncoder pivotEncoder = pivot.getAbsoluteEncoder();
  private final SparkMaxConfig pivotConfig;

  PIDController pivotPid = new PIDController(0.3, 0, 0);

  private boolean atMin, atMax;
  /** Creates a new Pivot. */
  public Pivot() {
    // configs
    pivotConfig = new SparkMaxConfig();
    pivotConfig.idleMode(IdleMode.kBrake);

    // convert to radians
    pivotConfig.encoder.positionConversionFactor((2 * Math.PI) / RollersConstants.intakeRollersRatio); // 1 rotation = 2 pi

    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // pid limits
    pivotPid.enableContinuousInput(-Math.PI, Math.PI);
    pivotPid.setTolerance(0.05);
    pivotPid.setIntegratorRange(-0.01, 0.01);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake/Pivot/Speed", pivot.get());
    SmartDashboard.putNumber("Intake/Pivot/PID/error", pivotPid.getError());
    SmartDashboard.putNumber("Intake/Pivot/PID/setpoint", pivotPid.getSetpoint());
    SmartDashboard.putNumber("Intake/Pivot/Abs Encoder deg", pivot.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("Intake/Pivot/current", getCurrent());
    SmartDashboard.putNumber("Intake/Pivot/Bus Voltage", getBusVoltage());
    SmartDashboard.putBoolean("Intake/Pivot/at max", atMax);
    SmartDashboard.putBoolean("Intake/Pivot/at min", atMin);
    pivotPid.reset();
  }
  
  public void setPivot(double speed) { 
    // clamp
    if (atMax && speed > 0) {
      speed = 0; stopPivot();
    } 
    if (atMin && speed < 0) {
      speed = 0; stopPivot();
    }
    set(speed);
  }

  /**
   * @param setpoint the radians to set the pivot
   */
  public void setPivotPid(double setpoint) {
    double pid = pivotPid.calculate(pivotEncoder.getPosition(), setpoint);
    // check if above setpoint clamp, and clamp pid speed
    setpoint = MathUtil.clamp(setpoint, PivotConstants.pivotMin, PivotConstants.pivotMax);
    pid = MathUtil.clamp(pid, -0.5, 0.5);

    SmartDashboard.putNumber("Intake/Pivot/PID/setpoint", setpoint);
    SmartDashboard.putNumber("Intake/Pivot/PID/calc", pid);
    setPivot(pid);
  }

  public double getPivotDegrees() {
    return Math.toDegrees(pivotEncoder.getPosition());
  }

  public double getCurrent() {
    return pivot.getOutputCurrent();
  }

  public double getBusVoltage(){
    return pivot.getBusVoltage();
  }

  public boolean atSetpoint(){
    return pivotPid.atSetpoint();
  }

  public void set(double speed) {
    pivot.set(speed);
  }

  /**
   * Stops the pivot motor
   */
  public void stopPivot() {
    pivot.stopMotor();
  }
}
