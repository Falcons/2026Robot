// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TurretConstants.MovementConstants;
import frc.robot.subsystems.Swerve.Swerve;

public class Movement extends SubsystemBase {

  private final Swerve swerve;

  private final SparkMax turret = new SparkMax(MovementConstants.turretCANID, MotorType.kBrushless);
  private final AbsoluteEncoder turretEncoder = turret.getAbsoluteEncoder();
  private final SparkMaxConfig turretConfig = new SparkMaxConfig();

  private final Servo leftHoodActuator = new Servo(MovementConstants.leftHoodActuatorPWM);
  private final Servo rightHoodActuator = new Servo(MovementConstants.rightHoodActuatorPWM);

  private final PIDController turretPID = new PIDController(0.05, 0, 0);

  private boolean atMax, atMin;

  /** Creates a new Movement. */
  public Movement(Swerve swerve) {
    this.swerve = swerve;

    
    // turret configs 2048 ticks per revolution, convert to radians, divide by gear ratio
    turretConfig.encoder.positionConversionFactor(2048 / Math.PI * 2 / MovementConstants.turretRatio); // 1 rotation = 2 pi
    turret.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // turretPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // check if max or min;
    atMax = turretEncoder.getPosition() >= MovementConstants.turretMaxRad;
    atMin = turretEncoder.getPosition() <= MovementConstants.turretMinRad;

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret/Movmement/Turret/Speed", turret.get());
    SmartDashboard.putNumber("Turret/Movmement/Turret/Absolute Encoder", turretEncoder.getPosition());
    SmartDashboard.putBoolean("Turret/Movment/Turret/at max", atMax);
    SmartDashboard.putBoolean("Turret/Movment/Turret/at min", atMin);
    SmartDashboard.putNumber("Turret/Movememnt/Hood/left actuators", leftHoodActuator.get());
    SmartDashboard.putNumber("Turret/Movememnt/Hood/right actuators", rightHoodActuator.get());
  }

  /**
   * call this to actually auto aim to the goal
   */
  public void autoAim() {
    double setpoint;
    boolean correctTag = false;
    for (int tagID : MovementConstants.hubTagIDs) {
      correctTag = LimelightHelpers.getFiducialID(LimelightConstants.turretLimelight) == tagID;
    }
    // if the april tag id is in the hub tags than use april tag with tx
    if (correctTag) {
      setpoint = LimelightHelpers.getTargetPose_CameraSpace(LimelightConstants.turretLimelight)[0];
    } else { 
      // if there is no april tag use bot position, 
      setpoint = getRelativeRad();
    }
    aimToSetpoint(setpoint);
  }

  /**
   * Aim with botpose
   */
  public void aimToSetpoint(double setpoint) {
    // clamp setpoint
    setpoint = MathUtil.clamp(setpoint, MovementConstants.turretMinRad, MovementConstants.turretMaxRad);
    // calc pid
    double pid = turretPID.calculate(turretEncoder.getPosition(), -setpoint);
    // clamp setpoint 
    pid = MathUtil.clamp(pid, -0.5, 0.5);

    // move motor
    setTurret(pid);
    turretPID.reset();
  }

  /**
   * move the turret wiht a clamp
   * @param speed the speed to move the turret
   */
  public void setTurret(double speed) { 
    // clamp
    if (atMax && speed > 0) {
      speed = 0; turret.stopMotor();
    } 
    if (atMin && speed < 0) {
      speed = 0; turret.stopMotor();
    }
    turret.set(speed);
  }

  public void setHood(double Position){
    leftHoodActuator.set(Position);
    rightHoodActuator.set(Position);
  }

  public double getLeftHoodPosition(){
    return leftHoodActuator.getPosition();
  }
  public double getRightHoodPosition(){
    return rightHoodActuator.getPosition();
  }
  public double getHoodPosition(){
    return (getLeftHoodPosition() + getRightHoodPosition())/2;
  }

  /**
   * @return the angle in which the shooter should be aiming at towards the goal in radians
   */
  public double getGlobalRad() {
    // Translation2d distanceToGoal = swerve.getPose().getTranslation().minus(AimerConstants.goalPos);
    Translation2d distanceToGoal = MovementConstants.goalPos.minus(swerve.getPose().getTranslation());
    // to get target angle use inverse tan O/A
    double targetAngle = Math.atan2(distanceToGoal.getY(), distanceToGoal.getX()); 
    
    return MathUtil.angleModulus(targetAngle);
  }

  /**
   * @return the relative abgle the shooter should point at in radians
   */
  public double getRelativeRad() {
    return MathUtil.angleModulus(swerve.getHeading().getRadians() - getGlobalRad());
  }

  /**
   * @return true if turret is in range
   */
  public boolean turretInRange() {
    return getRelativeRad() - turretEncoder.getPosition() < MovementConstants.turretError;
  }

  /**
   * @return true if hood is in range
   */
  public boolean hoodInRange() {
    return false;
  }

  /**
   * checks if the turret is in range
   * @return a bool, true if in range
   */
  public boolean inRange() {
    return turretInRange() && hoodInRange();
  }
}
