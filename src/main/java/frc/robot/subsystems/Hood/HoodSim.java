// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hood;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants.MovementConstants;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.subsystems.Turret.LaunchCalculator;
import frc.robot.subsystems.Turret.ShooterSim;

public class HoodSim extends SubsystemBase {

  private final Swerve swerve;
  private final ShooterSim shooterSim;
  private final Field2d field = new Field2d();
  
  private final Servo leftHoodActuatorSim = new Servo(MovementConstants.leftHoodActuatorPWM-10);
  private final Servo rightHoodActuatorSim = new Servo(MovementConstants.rightHoodActuatorPWM-10);

  // hood
  private Pose2d hoodPoseLeft = new Pose2d();
  private Pose2d hoodPoseRight = new Pose2d();

  /** Creates a new Movement. */
  public HoodSim(Swerve swerve, ShooterSim shooterSim) {
    // leftHoodActuator.createDouble("position", Direction.kBidir, 0);
    // rightHoodActuator.createDouble("position", Direction.kBidir, 0);
    this.swerve = swerve;
    this.shooterSim = shooterSim;
    SmartDashboard.putData("Field", field);
    // turretPID.enableContinuousInput(-Math.PI, Math.PI);
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

    SmartDashboard.putNumber("Hood/HoodSim/left actuators", leftHoodActuatorSim.getAngle());
    SmartDashboard.putNumber("Hood/HoodSim/right actuators", rightHoodActuatorSim.getAngle());
    SmartDashboard.putNumber("Hood/HoodSim/Auto hood angle", getHoodAngle());
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
    return LaunchCalculator.getInstance().getParameters(swerve, shooterSim.getShooterRPS()).hoodAngle();
  }

  /**
   * Set the hood to the position
   * @param Position position 
   */
  public void set(double Position){
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
   * @param speed degrees * 20  /second
   */
  public void move(double speed){
    leftHoodActuatorSim.setAngle(leftHoodActuatorSim.getAngle() + speed);
    leftHoodActuatorSim.setAngle(leftHoodActuatorSim.getAngle() + speed);
  }

  /**
   * Get the average of the two hood angles
   * @return the angle in degrees
   */
  public double getHoodPosition(){
    return (leftHoodActuatorSim.getAngle() + rightHoodActuatorSim.getAngle())/2;
  }

  /**
   * @return true if hood is in range
   */
  public boolean inRange() {
    return true;
  }
}
