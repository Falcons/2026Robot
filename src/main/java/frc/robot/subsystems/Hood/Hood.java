// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hood;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.subsystems.Turret.LaunchCalculator;
// import frc.robot.subsystems.Turret.Shooter.Shooter;

public class Hood extends SubsystemBase {

  private final Swerve swerve;
  // private final Shooter shooter;
  
  private final Servo leftHoodActuator = new Servo(HoodConstants.leftHoodActuatorPWM);
  private final Servo rightHoodActuator = new Servo(HoodConstants.rightHoodActuatorPWM);

  
  /** Creates a new Movement. */
  public Hood(Swerve swerve) {
    this.swerve = swerve;
    // this.shooter = shooter;

    leftHoodActuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    rightHoodActuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood/left actuators", getLeft());
    SmartDashboard.putNumber("Hood/right actuators", getRight());
    SmartDashboard.putNumber("Hood/actuators mean", get());
    SmartDashboard.putNumber("Hood/Auto hood angle", getHoodAngle());
  }

  public void autoAim(){
    double setpoint = MathUtil.clamp(getHoodAngle(), HoodConstants.hoodMin, HoodConstants.hoodMax);
    set(setpoint);
  }

  /**
   * @return the angle in which the hood should be aiming at 
   */
  public double getHoodAngle() {
    // use the launch calulator to get hood angle
    LaunchCalculator.getInstance().clearLaunchingParameters();
    return LaunchCalculator.getInstance().getParameters(swerve, -1.0).hoodAngle();
  }

  /**
   * Set the hood to the position 0 - 1
   * @param position position 
   */
  public void set(double position){
    leftHoodActuator.setPosition(position);
    rightHoodActuator.setPosition(position);
  }
  public void set(DoubleSupplier position){
    set(position.getAsDouble());
  }

  /**
   * Set the hood to the position
   * @param Position position in degrees
   */
  public void setDeg(double angle){
    double clampAngle = MathUtil.clamp(angle, 0, 180);
    leftHoodActuator.setAngle(clampAngle);
    rightHoodActuator.setAngle(clampAngle);
  }

  public void setDeg(DoubleSupplier angle){
    setDeg(angle.getAsDouble());
  }

  /**
   * move hood based on commanded position
   * @param speed % of power
   */
  public void moveHood(DoubleSupplier speed){
    set(get() + (speed.getAsDouble() * HoodConstants.hoodSpeedMultipier));
  }

  public double getLeft(){
    return leftHoodActuator.get();
  }
  public double getRight(){
    return rightHoodActuator.get();
  }
  public double get(){
    return (getLeft() + getRight())/2;
  }
}
