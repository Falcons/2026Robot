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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve.Swerve;
// import frc.robot.subsystems.Turret.Shooter.Shooter;

public class Turret extends SubsystemBase {

  private final Swerve swerve;
  // private final Shooter shooter;

  private final SparkMax turret = new SparkMax(TurretConstants.turretCANID, MotorType.kBrushless);
  private final AbsoluteEncoder turretEncoder = turret.getAbsoluteEncoder();
  private final SparkMaxConfig turretConfig = new SparkMaxConfig();

  private final PIDController turretPID = new PIDController(0.05, 0, 0);

  private boolean atMax, atMin;

  /** Creates a new Movement. */
  public Turret(Swerve swerve) {
    this.swerve = swerve;
    // this.shooter = shooter;

    turretConfig.absoluteEncoder.positionConversionFactor(Math.PI);
    turretConfig.absoluteEncoder.inverted(true);
    turretConfig.smartCurrentLimit(20);
    turretConfig.idleMode(IdleMode.kBrake);
    turret.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

  /* TODO: add to limelight
   * shoot Limelight
   * x 11.93
   * h 11.9 
   */

  @Override
  public void periodic() {
    Double id =  LimelightHelpers.getFiducialID(LimelightConstants.stillLimelight);

    // max saffty
    atMax = turretEncoder.getPosition() >= TurretConstants.turretMaxRad;
    atMin = turretEncoder.getPosition() <= TurretConstants.turretMinRad;

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret/Turret/idk", 5000);
    SmartDashboard.putNumber("Turret/Turret/Speed", turret.get());
    SmartDashboard.putNumber("Turret/Turret/Absolute Encoder Rad", getEncoderRad());
    SmartDashboard.putNumber("Turret/Turret/Absolute Encoder Deg", getEncoderDeg());
    SmartDashboard.putBoolean("Turret/Turret/at max", atMax);
    SmartDashboard.putBoolean("Turret/Turret/at min", atMin);
    // SmartDashboard.putNumber("Turret/Turret/global angle", Math.toDegrees(limelightRad()));
    // SmartDashboard.putNumber("Turret/Turret/distance", limelightDistance());
    SmartDashboard.putNumber("Turret/Turret/id", id);

    // set limelight pos in turret periodic
    Translation3d rotatedLimelightTranslation = LimelightConstants.turretLimelightPos;
    // Translation3d rotatedLimelightTranslation = LimelightConstants.turretLimelightPos.rotateAround(TurretConstants.robotToTurret.getTranslation(), new Rotation3d(0, 0, 90 - turretEncoder.getPosition()));
    LimelightHelpers.setCameraPose_RobotSpace(
      LimelightConstants.turretLimelight, 
      rotatedLimelightTranslation.getX(), 
      rotatedLimelightTranslation.getY(), 
      rotatedLimelightTranslation.getZ(),
      LimelightConstants.turretLimelightRot.getX(),
      LimelightConstants.turretLimelightRot.getY(),
      getEncoderDeg()-90);
  }

  /**
   * call this to actually auto aim to the goal
   */
  public void autoAim() {
    aimToSetpoint(getRelativeRad());
  }

  /**
   * Aim with setpoint
   */
  public void aimToSetpoint(double setpoint) { // TODO: pid needs to be done
    // clamp setpoint
    setpoint = MathUtil.clamp(setpoint, TurretConstants.turretMinRad, TurretConstants.turretMaxRad);
    // calc pid
    double pid = turretPID.calculate(turretEncoder.getPosition(), setpoint);
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
    turret.set(speed * 0.3);
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
    return MathUtil.angleModulus(swerve.getHeading().getRadians() - getGlobalRad());
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
   * get the angle absed on limelight
   *
  public double limelightRad() {
    if (lookingAtHub(LimelightConstants.turretLimelight)) {
        double aprilTagOffset[] = LimelightHelpers.getTargetPose_CameraSpace(LimelightConstants.turretLimelight);
        // get angle
        return (
          swerve.getHeading().getRadians() + // turret is facing where the bot is facing 
          Math.toRadians(90) - turretEncoder.getPosition() + // + its angle -90d offset
          Math.atan2(aprilTagOffset[1], aprilTagOffset[0]) //(TURRET ANGLE) + (APRIL TAG ANGLE)
        );
    }
    return 0.0;
  }*/

  /**
     * checks if the liemlight sees an aprul tag on the hub 
     * returns false always if in sim
     * @param limeligtName name of the limelight
     * @return true if limelight sees an april tag on the hub
     *
    public boolean lookingAtHub(String limeligtName) {
        System.out.println("looking at hub: called");
        if (RobotBase.isSimulation()) return false;
        
        // depending on team look at those limelights
        int tagIDs[] = LimelightConstants.blueHubTagIDs;
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            tagIDs = LimelightConstants.redHubTagIDs;
        }
        System.out.println("looking at hub: " + tagIDs);

        // if current tag is tag return true
        for (double tagID : tagIDs) {
            System.out.println("looking at hub: " + tagID);
            if (LimelightHelpers.getFiducialID(LimelightConstants.turretLimelight) == tagID) {
                return true;
            }
        }
        return false;
    } */
}
