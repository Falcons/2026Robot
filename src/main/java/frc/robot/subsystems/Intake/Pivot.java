// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.subsystems.Swerve.Swerve;

public class Pivot extends SubsystemBase {
  // pivot motor, encoder and config
  private final TalonFX pivot = new TalonFX(PivotConstants.pivotCANID); //kracken n44
  private final TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
  private final Swerve swerve;

  private final CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs();

  PIDController pivotPid = new PIDController(0.3, 0, 0);
  
  private boolean atMin, atMax;
  /** Creates a new Pivot. */
  public Pivot(Swerve swerve) {
    this.swerve = swerve;

    // smart current limits
    limitsConfigs.StatorCurrentLimit = 40;
    limitsConfigs.StatorCurrentLimitEnable = true;

    // configs
    pivotConfig.withCurrentLimits(limitsConfigs);
    pivotConfig.withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

    // pid limits
    pivotPid.setTolerance(0.05);
    pivotPid.setIntegratorRange(-0.01, 0.01);
  }
  
//   @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake/Pivot/Speed", pivot.get());
    SmartDashboard.putNumber("Intake/Pivot/PID/error", pivotPid.getError());
    SmartDashboard.putNumber("Intake/Pivot/PID/setpoint", pivotPid.getSetpoint());
    SmartDashboard.putNumber("Intake/Pivot/Abs Encoder deg", getPivotDegrees());
    SmartDashboard.putNumber("Intake/Pivot/Abs Encoder rad", getPivotRad());
    SmartDashboard.putBoolean("Intake/Pivot/at max", atMax);
    SmartDashboard.putBoolean("Intake/Pivot/at min", atMin);

    // set max and min
    atMax = getPivotRad() >= PivotConstants.pivotMax;
    atMin = getPivotRad() <= PivotConstants.pivotMin;
  }
  
  public void set(double speed) { 
    // clamp
    double clampSpeed = speed;
    if (atMax && speed < 0) {
      clampSpeed = 0; stopPivot();
    } 
    if (atMin && speed > 0) {
      clampSpeed = 0; stopPivot();
    }
    pivot.set(clampSpeed);
  }

  public void set(DoubleSupplier speed) { 
    set(speed.getAsDouble());
  }
  /**
   * @param setpoint the radians to set the pivot
   */
  public void setPivotPid(double setpoint) {
    double pid = pivotPid.calculate(getPivotRad(), setpoint);
    // check if above setpoint clamp, and clamp pid speed
    setpoint = MathUtil.clamp(setpoint, PivotConstants.pivotOut, PivotConstants.pivotIn); // pivot in is max
    pid = MathUtil.clamp(-pid, -0.5, 0.5);

    SmartDashboard.putNumber("Intake/Pivot/PID/setpoint", setpoint);
    SmartDashboard.putNumber("Intake/Pivot/PID/calc", pid);
    set(pid);
  }

  public boolean atSetpoint(){
    return pivotPid.atSetpoint();
  }

  /**
   * Stops the pivot motor
   */
  public void stopPivot() {
    pivot.stopMotor();
  }

  /**
   * returns the pivot encoder
   */
  public AbsoluteEncoder pivotEncoder() {
    SparkMax angleMotor = (SparkMax) swerve.getSwerveDrive().getModules()[3].getAngleMotor().getMotor();
    return angleMotor.getAbsoluteEncoder();
  }
  public double getPivotRad() {
    return getPivotDegrees() * Math.PI / 180;
  }
  public double getPivotDegrees() {
    return pivotEncoder().getPosition() * 360;
  }

  /**
   * reset pivot pid
   */
  public void resetPID() {
    pivotPid.reset();
  }
}
