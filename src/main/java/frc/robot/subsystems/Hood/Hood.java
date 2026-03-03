// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hood;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
  private final Field2d field = new Field2d();
  
  private final Servo leftHoodActuatorSim = new Servo(MovementConstants.leftHoodActuatorPWM);
  private final Servo rightHoodActuatorSim = new Servo(MovementConstants.rightHoodActuatorPWM);

  // simulated variables
  private Pose2d hoodPoseLeft = new Pose2d();
  private Pose2d hoodPoseRight = new Pose2d();

  /** Creates a new Movement. */
  public Hood(Swerve swerve, Shooter shooter) {
    this.swerve = swerve;
    this.shooter = shooter;
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    // hood
    hoodPoseLeft = new Pose2d(new Translation2d(5,0), 
                   new Rotation2d(Math.toRadians(leftHoodActuatorSim.getAngle())));
    hoodPoseRight = new Pose2d(new Translation2d(7,0), 
                   new Rotation2d(Math.toRadians(rightHoodActuatorSim.getAngle())));
    field.getObject("hoodLeft").setPose(hoodPoseLeft);
    field.getObject("hoodRight").setPose(hoodPoseRight);

    SmartDashboard.putNumber("Hood/left actuators", leftHoodActuatorSim.getAngle());
    SmartDashboard.putNumber("Hood/right actuators", rightHoodActuatorSim.getAngle());
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
    leftHoodActuatorSim.set(Position);
    rightHoodActuatorSim.set(Position);
  }

  /**
   * Set the hood to the position
   * @param Position position in degrees
   */
  public void setDeg(double Position){
    leftHoodActuatorSim.setAngle(Position);
    rightHoodActuatorSim.setAngle(Position);
  }

  /**
   * move hood based on commanded position + speed in degrees
   * @param speed degrees/second
   */
  public void moveHood(DoubleSupplier speed){
    leftHoodActuatorSim.setAngle(leftHoodActuatorSim.getAngle() + speed.getAsDouble()/Constants.deltaTime);
    leftHoodActuatorSim.setAngle(leftHoodActuatorSim.getAngle() + speed.getAsDouble()/Constants.deltaTime);
  }

  /**
   * Get the average of the two hood angles
   * @return the angle in degrees
   */
  public double getHoodPosition(){
    return (leftHoodActuatorSim.getAngle() + rightHoodActuatorSim.getAngle())/2;
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
