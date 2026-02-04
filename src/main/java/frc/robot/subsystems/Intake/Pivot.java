// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.Constants.IntakeConstants.RollersConstants;
import frc.robot.Constants.TurretConstants.MovementConstants;
import frc.robot.subsystems.Turret.Movement;

public class Pivot extends SubsystemBase {
  private final SparkMax pivot;
  private SparkMaxConfig pivotConfig;
  PIDController pivotPid = new PIDController(0.3, 0, 0);

  double previousCurrent = 0;
  public boolean atMin, atMax, cancelPID;
  /** Creates a new Pivot. */
  public Pivot() {
    this.pivot = new SparkMax(MovementConstants.pivotCANID, MotorType.kBrushless);
    pivotConfig = new SparkMaxConfig();
    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.encoder.positionConversionFactor(RollersConstants.intakeRollersRatio / 360); // 1 rotation = 360 degrees
    pivotConfig.encoder.positionConversionFactor(RollersConstants.intakeRollersRatio / 360); // 1 rotation = 360 degrees
    pivotConfig.smartCurrentLimit(30);
    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pivotPid.enableContinuousInput(-180, 180);
    pivotPid.setTolerance(0.05);
    pivotPid.setIntegratorRange(-0.01, 0.01);
  }
  public void stopPivot() {
    pivot.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot/PID/error", pivotPid.getError());
    SmartDashboard.putNumber("Pivot/PID/setpoint", pivotPid.getSetpoint());
    SmartDashboard.putNumber("Pivot/Abs Encoder", getAbsolute());
    SmartDashboard.putNumber("Pivot/Abs Encoder deg", getAbsEncoderDeg());
    SmartDashboard.putNumber("Pivot/current", getCurrent());
    SmartDashboard.putNumber("Pivot/Bus Voltage", getBusVoltage());
    SmartDashboard.putBoolean("Pivot/at max", atMax);
    SmartDashboard.putBoolean("Pivot/at min", atMin);
    pidReset();
  }

  public void pidReset() {
    pivotPid.reset();
  }
  public void setPivot(double speed) { 

    if (atMax && speed > 0){speed = 0; stopPivot();} // clamp
    if (atMin && speed < 0){speed = 0; stopPivot();} // clamp
  }
  public void setPivotPid(double setpoint) {
    double pid = pivotPid.calculate(getAbsolute(), setpoint);
    if(setpoint > PivotConstants.pivotMax) setpoint = PivotConstants.pivotMax;
    if(setpoint < PivotConstants.pivotMin) setpoint = PivotConstants.pivotMin;
    if(pid > 0.5) pid = 0.5;
    if(pid < -0.5) pid = -0.5;
    SmartDashboard.putNumber("Pivot/PID/error", pivotPid.getError());
    SmartDashboard.putNumber("Pivot/PID/setpoint", setpoint);
    SmartDashboard.putNumber("Pivot/PID/calc", pid);
    setPivot(pid);
  }
  public double getReletive() {
    return pivot.getEncoder().getPosition();
  }
  public double getAbsolute(){
    return pivot.getAbsoluteEncoder().getPosition()*2 * Math.PI - 2.431;
  }
  public double getAbsEncoderDeg(){
    return getAbsolute()*180.0/Math.PI;
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
}
