// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret.Shooter;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants.ShooterConstants;

public class Transfer extends SubsystemBase {

  private final TalonFX transfer = new TalonFX(ShooterConstants.transferCANID);
  private final TalonFXConfiguration transferConfig = new TalonFXConfiguration();
  private final CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs();

  private Timer timer = new Timer();

  /** Creates a new Transfer. */
  public Transfer() {
    timer.start();
    
    limitsConfigs.StatorCurrentLimit = 40;

    transferConfig.withCurrentLimits(limitsConfigs);
    transferConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    transfer.getConfigurator().apply(transferConfig);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret/Shooter/transferSpeed", transfer.get());
  }

  /**
   * Pulse the transfer motor in half a second intravals
   */
  public void pulse() {
    // move for half a second stop the other half
    if (timer.hasElapsed(2)) {
      transfer.set(-ShooterConstants.maxTransferSpeed);
    } else{
      transfer.set(ShooterConstants.maxTransferSpeed);
    }
    
    // reset timer
    if (timer.hasElapsed(2.5)) {
      timer.reset();
    }
  }

  public void pulse(BooleanSupplier run) {
    if (run.getAsBoolean()) {
      pulse();
    } else {
      transfer.set(0.0);
    }
  }
  
  /**
   * set the transfer speed
   * @param speed the speed to set
   */
  public void set(double speed) {
    transfer.set(speed);
  }
  /**
   * stop the transfer motoe
   */
  public void stop() {
    transfer.stopMotor();
  }
}
