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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightConstants;
import frc.robot.Constants.IntakeConstants.RollersConstants;
import frc.robot.subsystems.Lights;

public class Rollers extends SubsystemBase {
  private final Lights lights;

  private final TalonFX roller = new TalonFX(RollersConstants.rollerCANID);
  // configs
  private TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
  private final CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs();

  private final PIDController speedControl = new PIDController(0.9, 0.5, 0.0015);

  /** Creates a new Rollers. */
  public Rollers(Lights lights) {
    this.lights = lights;

    // configs
    rollerConfig.withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
    // smart current limits
    limitsConfigs.StatorCurrentLimit = 40; //TODO: current limite
    limitsConfigs.StatorCurrentLimitEnable = true;
    rollerConfig.withCurrentLimits(limitsConfigs);

    roller.getConfigurator().apply(rollerConfig);
  }

  @Override
  public void periodic() {
    if (roller.get() == 0) lights.removeQueue(LightConstants.intakePriority);
    SmartDashboard.putNumber("Intake/Rollers/Roller speed", roller.get());
    SmartDashboard.putNumber("Intake/Rollers/RPS", roller.getVelocity().getValueAsDouble());
  }

  /**
   * set rollers
   */
  public void set(double roll) {
    roller.set(roll);
    lights.addQueue(LightConstants.intakePriority);
  }

  public void setRPS(double RPS){ //TODO: RPS
    double pid = speedControl.calculate(getRPM(), RPS);
    SmartDashboard.putNumber("Intake/Rollers/PID/raw pid", pid);
    pid /= 105; // max RPs
    SmartDashboard.putNumber("Intake/Rollers/PID/adjusted pid", pid);
    set(roller.get() + pid);
  }
  public void setRPS(DoubleSupplier RPS){
    setRPS(RPS.getAsDouble());
  }

  public double getRPM(){
    return roller.getVelocity().getValueAsDouble();
  }

  /**
   * stop rollers
   */
  public void stop() {
    roller.stopMotor();
  }
}
