// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hood;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants.MovementConstants;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.subsystems.Turret.LaunchCalculator;
import frc.robot.subsystems.Turret.Shooter;

public class Hood extends SubsystemBase {

  private final Swerve swerve;
  private final Shooter shooter;
  
  private final Servo HoodActuator = new Servo(MovementConstants.HoodActuatorPWM);

  /** Creates a new Movement. */
  public Hood(Swerve swerve, Shooter shooter) {
    this.swerve = swerve;
    this.shooter = shooter;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood/left actuators", HoodActuator.getAngle());
    SmartDashboard.putNumber("Hood/Auto hood angle", getHoodAngle());
  }

  public void autoAim(){
    double setpoint = MathUtil.clamp(getHoodAngle(), 0, 180);
    setDeg(setpoint);
  }

  /**
   * @return the angle in which the hood should be aiming at 
   */
  public double getHoodAngle() {
    // use the launch calulator to get hood angle
    LaunchCalculator.getInstance().clearLaunchingParameters();
    return  Math.toDegrees(LaunchCalculator.getInstance().getParameters(swerve, shooter.getShooterRPS()).hoodAngle());
  }

  /**
   * Set the hood to the position
   * @param Position position 
   */
  public void setRad(double Position){
    HoodActuator.set(Position);
  }

  /**
   * Set the hood to the position
   * @param Position position in degrees
   */
  public void setDeg(double Position){
    HoodActuator.setAngle(Position);
  }

  /**
   * move hood based on commanded position + speed in degrees
   * @param speed degrees/second
   */
  public void moveHood(DoubleSupplier speed){
    HoodActuator.setAngle(HoodActuator.getAngle() + speed.getAsDouble()/Constants.deltaTime);
  }

  /**
   * checks if the hood is far away from the trench
   * @return true if its safe to move hood
   */
  public boolean awayFromTrench() {
    if (swerve.getPose().getX() < MovementConstants.hoodDownDistanceMinM && swerve.getPose().getX() > MovementConstants.hoodDownDistanceMaxM) {
      return true;
    }
    return false;
  }
}
