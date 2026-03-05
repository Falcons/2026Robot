// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants.ShooterConstants;
import frc.robot.subsystems.Turret.TurretSim;

public class ShooterSim extends SubsystemBase {
  
  // once shooters AND kicker are max speed than transfer
  private double leftShooter = 0;
  private double rightShooter = 0;
  private double kicker = 0;

  private double transfer = 0;
  private Timer timer = new Timer();
  public Timer shotTimer = new Timer();

  // need to connect to movement to see if its aligned
  private final TurretSim aimerSim;

  // variables
  public boolean shooterRunning = false;

  /** Creates a new Shooter. */
  public ShooterSim(TurretSim aimerSim) {
    timer.start();
    this.aimerSim = aimerSim;
  }

  /**
   * turns on transfer motor when the shooter reaches max speed
   */
  public void shootWhenMaxSpeed() {
    // dont run if not in range
    if (!aimerSim.inRange()) return;
       rightShooter += ShooterConstants.maxShooterSpeed;
       leftShooter += ShooterConstants.maxShooterSpeed;
      kicker += ShooterConstants.maxKickerSpeed;
    // wait until shooter is max speed than rotate transfer
    if (rightShooter >= ShooterConstants.maxShooterRPS) {
      pulseTransfer();
      shooterRunning = true;
      if(!shotTimer.isRunning()) shotTimer.start();
    }
  }

  /**
   * Pulse the transfer motor in half a second intravals
   */
  public void pulseTransfer() {
    // move for half a second stop the other half
    if (timer.hasElapsed(0.5)) {
      // transfer equal 0 hi
    } else{
      transfer += 1;
    }
    
    // reset timer
    if (timer.hasElapsed(1)) {
      timer.reset();
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret/ShooterSim/leftShooterSpeed", leftShooter);
    SmartDashboard.putNumber("Turret/ShooterSim/rightShooterSpeed", rightShooter);
    SmartDashboard.putNumber("Turret/ShooterSim/transferSpeed", transfer);
    SmartDashboard.putNumber("Turret/ShooterSim/kickerSpeed", kicker);
    SmartDashboard.putBoolean("Turret/ShooterSim/shooterRunning", shooterRunning);

    if (shotTimer.get() >= 2) {
      rightShooter = MathUtil.clamp(rightShooter - shotTimer.get(), 0, 150);
      leftShooter = MathUtil.clamp(leftShooter - shotTimer.get(), 0, 150);
      kicker = MathUtil.clamp(kicker - shotTimer.get(), 0, 150);
      transfer = MathUtil.clamp(transfer - shotTimer.get(), 0, 150);
    } else {
      rightShooter = MathUtil.clamp(rightShooter - 0.5, 0, 150);
      leftShooter = MathUtil.clamp(leftShooter - 0.5, 0, 150);
      kicker = MathUtil.clamp(kicker - 0.5, 0, 150);
      transfer = MathUtil.clamp(transfer - 0.5, 0, 150);
    }
  }

  /**
   * check if the shooter rpm is too low to shoot
   * @return true when rpm is too low
   */
  public boolean shooterRPMlow() {
    if (shooterRunning && rightShooter < ShooterConstants.maxShooterRPS) {
      shooterRunning = false;
      shotTimer.stop();
      shotTimer.reset();
      return true;
    }
    return false;
  }

  /**
   * set the transfer speed
   * @param speed the speed to set
   */
  public void setTransfer(double speed) {
    transfer += speed;
  }
  /**
   * set the kicker speed
   * @param speed the speed to set
   */
  public void setKicker(double speed) {
    kicker += speed;
  }
  /**
   * set the shooter speed
   * @param speed the speed to set
   */
  public void setShooter(DoubleSupplier speed) {
    rightShooter += speed.getAsDouble();
  }
  /**
   * set the shooter and transfer speed
   * @param shooterSpeed transfer speed
   * @param transferSpeed shooter speed
   * @param kickerSpeed kciker speed
   */
  public void fullShoot(DoubleSupplier shooterSpeed, double transferSpeed, double kickerSpeed) {
    rightShooter += shooterSpeed.getAsDouble();
    transfer += transferSpeed;
    kicker += kickerSpeed;
  }
  /**
   * set the shooter and kicker speed
   * @param shooterSpeed shooter speed
   * @param kickerSpeed kciker speed
   */
  public void setShooterWithkicker(DoubleSupplier shooterSpeed, double kickerSpeed) {
    rightShooter += shooterSpeed.getAsDouble();
    kicker = kickerSpeed;
  }

  /**
   * get the rpm of the shooter
   */
  public Double getShooterRPS() {
    return rightShooter;
  }
}
