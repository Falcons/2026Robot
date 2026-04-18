// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret.Shooter;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightConstants;
import frc.robot.Constants.TurretConstants.ShooterConstants;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Turret.Turret;

public class Transfer extends SubsystemBase {

  private final Turret turret;
  private Shooter shooter;
  private final Lights lights;

  private final TalonFX transfer = new TalonFX(ShooterConstants.transferCANID);
  private final TalonFXConfiguration transferConfig = new TalonFXConfiguration();
  private final CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs();

  private final PIDController speedControl = new PIDController(0.9, 0.5, 0.0015);

  private Timer timer = new Timer();

  /** Creates a new Transfer. */
  public Transfer(Turret turret, Lights lights) {
    this.turret = turret;
    this.lights = lights;
    timer.start();
    
    // pid
    var slot0Configs = new Slot0Configs(); // shouldnt need a
    slot0Configs.kS = 0.083585; // Add 0.25 V output to overcome static friction 0.4
    slot0Configs.kV = 0.11406; // A velocity target of 1 rps results in 0.12 V output 0.11
    slot0Configs.kP = 2; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0.0; // no output for integrated error
    slot0Configs.kD = 0.5; // A velocity error of 1 rps results in 0.1 V output

    limitsConfigs.StatorCurrentLimit = 40;

    transferConfig.withSlot0(slot0Configs);
    transferConfig.withCurrentLimits(limitsConfigs);
    transferConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    transfer.getConfigurator().apply(transferConfig);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret/Shooter/Transfer/volocity", transfer.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Turret/Shooter/Transfer/supplyCurrent", transfer.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Turret/Shooter/Transfer/supplyVoltage", transfer.getSupplyVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Turret/Shooter/Transfer/speed", transfer.get());

    if (transfer.get() == 0) {
      lights.removeQueue(LightConstants.manualTransferPriority);
    }
  }

  /**
   * set the transfer speed
   * @param speed the speed to set
   */
  public void set(double speed) {
    transfer.set(speed);
    lights.addQueue(LightConstants.manualTransferPriority);
  }
  public void set(BooleanSupplier run, double speed) {
    if(run.getAsBoolean()) set(speed);
    else stop();
  }

  /**
   * set the speed of the motor in rps
   * @param speed rps
   */
  public void setRps(double speed){
    var request = new VelocityVoltage(0).withSlot(0);
    transfer.setControl(request.withVelocity(speed));
    SmartDashboard.putNumber("Turret/Shooter/Transfer/PID/cc error", transfer.getClosedLoopError().getValueAsDouble());
  }

  public void pulse(){
    if(timer.hasElapsed(2))set(-ShooterConstants.maxTransferSpeed);
    else set(ShooterConstants.maxTransferSpeed);

    if(timer.hasElapsed(2.2)) timer.reset();
  }
  public void pulse(BooleanSupplier run){
    if(run.getAsBoolean()) pulse();
    else stop();
  }
  /**
   * stop the transfer motor
   */
  public void stop() {
    transfer.stopMotor();
  }

  public void autoTransfer() {
    if (!turret.inRange()) return;
    if (shooter.atTargetRps()) {
      setRps(ShooterConstants.maxTransferSpeedRPS);
    }
  }

  public void autoPulseTransfer() {
    if (!turret.inRange()) return;
    pulse(() -> shooter.atTargetRps());
  }

  public void setShooter(Shooter shooter) {
    this.shooter = shooter;
  }

}
