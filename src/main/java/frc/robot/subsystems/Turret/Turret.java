// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TurretConstants.MovementConstants;
import frc.robot.subsystems.Swerve.Swerve;

public class Turret extends SubsystemBase {

  private final Swerve swerve;
  private final Shooter shooter;

  private final SparkMax turret = new SparkMax(MovementConstants.turretCANID, MotorType.kBrushless);
  private final AbsoluteEncoder turretEncoder = turret.getAbsoluteEncoder();
  private final SparkMaxConfig turretConfig = new SparkMaxConfig();

  private final PIDController turretPID = new PIDController(0.05, 0, 0);

  private boolean atMax, atMin;

  /** Creates a new Movement. */
  public Turret(Swerve swerve, Shooter shooter) {
    this.swerve = swerve;
    this.shooter = shooter;

    // turret configs 2048 ticks per revolution, convert to radians, divide by gear ratio
    turretConfig.encoder.positionConversionFactor(Math.PI); // 360 degree of absolute = 180 degree on turret, convert to radians
    turret.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // turretPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // check if max or min;
    atMax = turretEncoder.getPosition() >= MovementConstants.turretMaxRad;
    atMin = turretEncoder.getPosition() <= MovementConstants.turretMinRad;

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret/Turret/Speed", turret.get());
    SmartDashboard.putNumber("Turret/Turret/Absolute Encoder", turretEncoder.getPosition());
    SmartDashboard.putBoolean("Turret/Turret/at max", atMax);
    SmartDashboard.putBoolean("Turret/Turret/at min", atMin);

    //TODO: set limelight pos in turret periodic
    // LimelightHelpers.setCameraPose_RobotSpace(
    //   LimelightConstants.turretLimelight, 
    //   getLimelightPos()[1], 
    //   getLimelightPos()[0], 
    //   LimelightConstants.up, 
    //   LimelightConstants.roll, 
    //   LimelightConstants.pitch,
    //   LimelightConstants.yaw);
  }

  /**
   * call this to actually auto aim to the goal
   */
  public void autoAim() {
    double setpoint;
    boolean correctTag = false;
    // loop through tags to set correct tag //TODO:LIMELIGHT
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
    set(pid);
    turretPID.reset();
  }

  /**
   * set the speed of the turret
   * @param speed of turret
   */
  public void set(DoubleSupplier speed) {
    set(speed.getAsDouble());
  }

  /**
   * move the turret wiht a clamp
   * @param speed the speed to move the turret
   */
  public void set(double speed) { 
    // clamp
    if (atMax && speed > 0) {
      speed = 0; turret.stopMotor();
    } 
    if (atMin && speed < 0) {
      speed = 0; turret.stopMotor();
    }
    turret.set(speed);
  }
  /**
   * @return the angle in which the shooter should be aiming at towards the goal in radians
   */
  public double getGlobalRad() {
    // use the launch calulator to get global angle
    LaunchCalculator.getInstance().clearLaunchingParameters();
    return LaunchCalculator.getInstance().getParameters(swerve, shooter.getShooterRPS()).turretAngle().getRadians();
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
  public boolean inRange() {
    return getRelativeRad() - turretEncoder.getPosition() < MovementConstants.turretError;
  }

  /**
   * Gets the position of the limelight based on turret angle
   * @return x, y relative to the robot x, y
   */
  public Double[] getLimelightPos() {
    return new Double[] {
      LimelightConstants.xOffset + LimelightConstants.radius * Math.cos(turret.getAbsoluteEncoder().getPosition()),
      LimelightConstants.yOffset + LimelightConstants.radius * Math.sin(turret.getAbsoluteEncoder().getPosition())
    };
  }
}
