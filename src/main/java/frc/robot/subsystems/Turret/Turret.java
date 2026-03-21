// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve.Swerve;
// import frc.robot.subsystems.Turret.Shooter.Shooter;

public class Turret extends SubsystemBase {

  private final Swerve swerve;
  // private final Shooter shooter;
  private final Lights lights;

  private final SparkMax turret = new SparkMax(TurretConstants.turretCANID, MotorType.kBrushless);
  private final AbsoluteEncoder turretEncoder = turret.getAbsoluteEncoder();
  private final SparkMaxConfig turretConfig = new SparkMaxConfig();

  private final PIDController turretPID = new PIDController(0.6, 0, 0);

  private boolean atMax, atMin;

  /** Creates a new Movement. */
  public Turret(Swerve swerve, Lights lights) {
    this.swerve = swerve;
    this.lights = lights;
    // this.shooter = shooter;

    turretConfig.absoluteEncoder.positionConversionFactor(Math.PI);
    turretConfig.absoluteEncoder.inverted(true);
    turretConfig.smartCurrentLimit(20);
    turretConfig.idleMode(IdleMode.kBrake);
    turret.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

    if (Math.abs(turret.get()) > 0) {
      lights.addQueue(LightConstants.autoFireAimPriority);
    } else {
      lights.removeQueue(LightConstants.autoFireAimPriority);
    }

    // max saffty
    atMax = turretEncoder.getPosition() >= TurretConstants.turretMaxRad;
    atMin = turretEncoder.getPosition() <= TurretConstants.turretMinRad;

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret/Turret/Speed", turret.get());
    SmartDashboard.putNumber("Turret/Turret/Absolute Encoder Rad", getEncoderRad());
    SmartDashboard.putNumber("Turret/Turret/Absolute Encoder Deg", getEncoderDeg());
    SmartDashboard.putBoolean("Turret/Turret/at max", atMax);
    SmartDashboard.putBoolean("Turret/Turret/at min", atMin);
    
    // limelight
    // SmartDashboard.putNumber("Turret/Turret/Limelight/TX", LimelightHelpers.getTargetPose_RobotSpace(LimelightConstants.turretLimelight)[0]);
    // SmartDashboard.putNumber("Turret/Turret/Limelight/TY", LimelightHelpers.getTargetPose_RobotSpace(LimelightConstants.turretLimelight)[1]);
    // SmartDashboard.putNumber("Turret/Turret/Limelight/TZ", LimelightHelpers.getTargetPose_RobotSpace(LimelightConstants.turretLimelight)[2]);

    // MATH
    SmartDashboard.putNumber("Turret/Turret/MATH/global deg", Math.toDegrees(limelightGlobalRad()));
    SmartDashboard.putNumber("Turret/Turret/MATH/rel deg", Math.toDegrees(limelightRelativeRad()));
    // launch calculator
    SmartDashboard.putNumber("Turret/Turret/LaunchCalc/global deg", Math.toDegrees(getGlobalRad()));
    SmartDashboard.putNumber("Turret/Turret/LaunchCalc/rel deg", Math.toDegrees(getRelativeRad()));

    // set limelight pos in turret periodic
    Translation3d rotatedLimelightTranslation = LimelightConstants.turretLimelightPos.rotateAround(TurretConstants.robotToTurret.getTranslation(), new Rotation3d(0, 0, -(turretEncoder.getPosition() - Math.toRadians(90))));
    
    LimelightHelpers.setCameraPose_RobotSpace(
      LimelightConstants.turretLimelight, 
      rotatedLimelightTranslation.getX(), 
      -rotatedLimelightTranslation.getY(), 
      rotatedLimelightTranslation.getZ(),
      Math.toDegrees(LimelightConstants.turretLimelightRot.getX()),
      Math.toDegrees(LimelightConstants.turretLimelightRot.getY()),
      -getEncoderDeg()+90);
    
  }

  /**
   * call this to actually auto aim to the goal
   */
  public void autoAim() {// swap to launchCalc
    aimToSetpoint(getRelativeRad());
  }

  /**
   * call this to actually auto aim to the goal
   */
  public void autoLimelightAim() {
    if (!LimelightHelpers.lookingAtHub(LimelightConstants.turretLimelight)) {
      turret.set(0);
    } else aimToSetpoint(limelightRelativeRad());
  }

  /**
   * Aim with setpoint
   */
  public void aimToSetpoint(double setpoint) {
    // clamp setpoint
    setpoint = MathUtil.clamp(setpoint, TurretConstants.turretMinRad, TurretConstants.turretMaxRad);
    // calc pid
    double pid = turretPID.calculate(turretEncoder.getPosition(), setpoint);
    SmartDashboard.putNumber("Turret/Turret/PID/error", pid);
    // clamp setpoint 
    pid = MathUtil.clamp(pid, -0.5, 0.5);
    // move motor
    set(pid);
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
      speed = 0;
    } 
    if (atMin && speed < 0) {
      speed = 0;
    }
    turret.set(speed);
  }

  public void stop(){
    turret.stopMotor();
  }
  /**
   * @return the angle in which the shooter should be aiming at towards the goal in radians
   */
  public double getGlobalRad() {
    // use the launch calulator to get global angle
    LaunchCalculator.getInstance().clearLaunchingParameters();
    return LaunchCalculator.getInstance().getParameters(swerve, turretEncoder.getPosition()).turretAngle().getRadians();
  }

  /**
   * @return the relative abgle the shooter should point at in radians
   */
  public double getRelativeRad() {
    return MathUtil.angleModulus(
      swerve.getHeading().getRadians() - getGlobalRad() + Math.toRadians(90)); // + + -
      // TurretConstants.turretMinRad, TurretConstants.turretMaxRad);
    // return MathUtil.angleModulus(
    //   (swerve.getHeading().getRadians() - getGlobalRad() - Math.toRadians(270)) * -1);
  }
  
  public double getEncoderRad(){
    return  turretEncoder.getPosition();
  }
  public double getEncoderDeg(){
    return Math.toDegrees(turretEncoder.getPosition());
  }

  /**
   * @return true if turret is in range
   */
  public boolean inRange() {
    return Math.abs(getRelativeRad() - turretEncoder.getPosition()) < TurretConstants.turretError;
  }

  /**
   * @return true if turret is at pid setpoit
   */
  public boolean atSetpoint() {
    return turretPID.atSetpoint();
  }

  /**
   * reset turret pid
   */
  public void resetPID() {
    turretPID.reset();
  }

  /**
   * get the angle based on limelight
   */
  public double limelightGlobalRad() {
    if (!LimelightHelpers.lookingAtHub(LimelightConstants.turretLimelight)) return 0.0;

    double aprilTagOffset[] = LimelightHelpers.getTargetPose_CameraSpace(LimelightConstants.turretLimelight);
    // get angle
    return MathUtil.angleModulus(
      swerve.getHeading().getRadians() - // + // turret is facing where the bot is facing 
      (turretEncoder.getPosition() - Math.toRadians(90)) - // + its angle -90d offset
      Math.atan2(aprilTagOffset[0], aprilTagOffset[2]) //(TURRET ANGLE) + (APRIL TAG ANGLE)
      // + Math.toRadians(180)
    );
  }

  /**
   * get the angle based on limelight with offsets
   */
  public double limelightOffsetGlobalRad() {
    if (!LimelightHelpers.lookingAtHub(LimelightConstants.turretLimelight)) return 0.0;

    double tagID = LimelightHelpers.getFiducialID(LimelightConstants.turretLimelight);

    // get april tag pose
    // AprilTagFieldLayout layout = AprilTagFields.k2026RebuiltWelde
    AprilTagFieldLayout layout = AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();
    Optional<Pose3d> tagPose = layout.getTagPose((int)tagID);

    Translation2d tagGlobalPos = null;
    if (tagPose.isPresent()) {
        tagGlobalPos = tagPose.get().getTranslation().toTranslation2d();
        tagGlobalPos = new Translation2d(
          tagGlobalPos.getX() + LimelightConstants.tagOffsets.get(tagID).getX(),
          tagGlobalPos.getY() + LimelightConstants.tagOffsets.get(tagID).getY());
    } else {
      return 0.0;
    }

    // get angle
    return MathUtil.angleModulus(
      Math.atan2(
        tagGlobalPos.getY() - swerve.getPose().getY(),
        tagGlobalPos.getX() - swerve.getPose().getX()
      ));
  }

  public double limelightRelativeRad() {
    if (!LimelightHelpers.lookingAtHub(LimelightConstants.turretLimelight)) return 0.0;

    return MathUtil.clamp(MathUtil.angleModulus(
      swerve.getHeading().getRadians() - limelightGlobalRad() + Math.toRadians(90)), // + + -
      TurretConstants.turretMinRad, TurretConstants.turretMaxRad);
  }
}
