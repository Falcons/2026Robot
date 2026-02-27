// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants.ShooterConstants;

public class Shooter extends SubsystemBase {
  
  // once shooters AND kicker are max speed than transfer
  private final TalonFX leftShooter = new TalonFX(ShooterConstants.leftShooterCANID); // Kraken x60
  private final TalonFX rightShooter = new TalonFX(ShooterConstants.rightShooterCANID);
  private final TalonFX kicker = new TalonFX(ShooterConstants.kickerCANID);

  private final TalonFX transfer = new TalonFX(ShooterConstants.transferCANID);
  private Timer timer = new Timer();

  // configs
  private final TalonFXConfiguration leftShooterConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration rightShooterConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration kickerConfig = new TalonFXConfiguration();

  private final TalonFXConfiguration transferConfig = new TalonFXConfiguration();

  // need to connect to movement to see if its aligned
  private final Turret aimer;

  // variables
  public boolean shooterRunning = false;

  /** Creates a new Shooter. */
  public Shooter(Turret aimer) {
    timer.start();
    this.aimer = aimer;

    // follow right shooter
    leftShooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftShooter.setControl(new Follower(ShooterConstants.rightShooterCANID, MotorAlignmentValue.Opposed));
    kicker.setControl(new Follower(ShooterConstants.rightShooterCANID, MotorAlignmentValue.Aligned));

    // apply configs
    leftShooter.getConfigurator().apply(leftShooterConfig);
    rightShooter.getConfigurator().apply(rightShooterConfig);
    transfer.getConfigurator().apply(transferConfig);
    kicker.getConfigurator().apply(kickerConfig);
  }

  /**
   * turns on transfer motor when the shooter reaches max speed
   */
  public void shootWhenMaxSpeed() {
    // dont run if not in range
    if (!aimer.inRange()) return;

    rightShooter.set(ShooterConstants.maxShooterSpeed);
    kicker.set(ShooterConstants.maxKickerSpeed);
    // wait until shooter is max speed than rotate transfer
    if (rightShooter.getVelocity().getValueAsDouble() >= ShooterConstants.maxShooterRPS) {
      pulseTransfer();
      shooterRunning = true;
    }
  }

  /**
   * Pulse the transfer motor in half a second intravals
   */
  public void pulseTransfer() {
    // move for half a second stop the other half
    if (timer.hasElapsed(0.5)) {
      transfer.set(0);
    } else{
      transfer.set(ShooterConstants.maxTransferSpeed);
    }
    
    // reset timer
    if (timer.hasElapsed(1)) {
      timer.reset();
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret/Shooter/leftShooterSpeed", leftShooter.get());
    SmartDashboard.putNumber("Turret/Shooter/rightShooterSpeed", rightShooter.get());
    SmartDashboard.putNumber("Turret/Shooter/transferSpeed", transfer.get());
    SmartDashboard.putNumber("Turret/Shooter/kickerSpeed", kicker.get());

    SmartDashboard.putNumber("Turret/Shooter/rightShooterVelocity", rightShooter.getVelocity().getValueAsDouble());
  }

  /**
   * check if the shooter rpm is too low to shoot
   * @return true when rpm is too low
   */
  public boolean shooterRPMlow() {
    if (shooterRunning && rightShooter.getVelocity().getValueAsDouble() < ShooterConstants.maxShooterRPS) {
      shooterRunning = false;
      return true;
    }
    return false;
  }

  /**
   * set the transfer speed
   * @param speed the speed to set
   */
  public void setTransfer(double speed) {
    transfer.set(speed);
  }
  /**
   * set the kicker speed
   * @param speed the speed to set
   */
  public void setKicker(double speed) {
    kicker.set(speed);
  }
  /**
   * set the shooter speed
   * @param speed the speed to set
   */
  public void setShooter(DoubleSupplier speed) {
    rightShooter.set(speed.getAsDouble());
  }
  /**
   * set the shooter and transfer speed
   * @param shooterSpeed transfer speed
   * @param transferSpeed shooter speed
   * @param kickerSpeed kciker speed
   */
  public void fullShoot(DoubleSupplier shooterSpeed, double transferSpeed, double kickerSpeed) {
    rightShooter.set(shooterSpeed.getAsDouble());
    transfer.set(transferSpeed);
    kicker.set(kickerSpeed);
  }
  /**
   * set the shooter and kicker speed
   * @param shooterSpeed shooter speed
   * @param kickerSpeed kciker speed
   */
  public void setShooterWithkicker(DoubleSupplier shooterSpeed, double kickerSpeed) {
    rightShooter.set(shooterSpeed.getAsDouble());
    kicker.set(kickerSpeed);
  }
}
