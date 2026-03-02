// Copyright (c) 2025-2026 Littleton Robotics
// https://github.com/Mechanical-Advantage/RobotCode2026Public < go here to steal yummy code
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.Turret;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants.MovementConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Util.AllianceFlipUtil;
import frc.robot.Util.FieldConstants;
import frc.robot.subsystems.Swerve.Swerve;

public class LaunchCalculator {
  private static LaunchCalculator instance;

  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.deltaTime));
  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.deltaTime));

  private Rotation2d lastTurretAngle;
  private double lastHoodAngle;
  private Rotation2d turretAngle;
  private double hoodAngle = Double.NaN;
  private double turretVelocity;
  private double hoodVelocity;

  public static LaunchCalculator getInstance() {
    if (instance == null) instance = new LaunchCalculator();
    return instance;
  }

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed) {}

  // Cache parameters
  private LaunchingParameters latestParameters = null;

  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;
  private static final InterpolatingTreeMap<Double, Rotation2d> launchHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap launchFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 1.34;
    maxDistance = 5.6;
    phaseDelay = 0.03;

    launchHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));
    launchHoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
    launchHoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
    launchHoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
    launchHoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
    launchHoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
    launchHoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
    launchHoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
    launchHoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
    launchHoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));

    launchFlywheelSpeedMap.put(1.34, 210.0);
    launchFlywheelSpeedMap.put(1.78, 220.0);
    launchFlywheelSpeedMap.put(2.17, 220.0);
    launchFlywheelSpeedMap.put(2.81, 230.0);
    launchFlywheelSpeedMap.put(3.82, 250.0);
    launchFlywheelSpeedMap.put(4.09, 255.0);
    launchFlywheelSpeedMap.put(4.40, 260.0);
    launchFlywheelSpeedMap.put(4.77, 265.0);
    launchFlywheelSpeedMap.put(5.57, 275.0);
    launchFlywheelSpeedMap.put(5.60, 290.0);

    // Existing entries
    timeOfFlightMap.put(1.38, 0.90);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(5.68, 1.16);
  }

  public LaunchingParameters getParameters(Swerve swerve) {
    if (latestParameters != null) {
      return latestParameters;
    }

    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = swerve.getPose();

    ChassisSpeeds robotRelativeVelocity = swerve.getFieldVelocity();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate distance from turret to target
    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    Pose2d turretPosition = estimatedPose.transformBy(toTransform2d(TurretConstants.robotToTurret));
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    ChassisSpeeds robotVelocity = swerve.getFieldVelocity();
    double robotAngle = estimatedPose.getRotation().getRadians();
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (TurretConstants.robotToTurret.getY() * Math.cos(robotAngle)
                    - TurretConstants.robotToTurret.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (TurretConstants.robotToTurret.getX() * Math.cos(robotAngle)
                    - TurretConstants.robotToTurret.getY() * Math.sin(robotAngle));

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;
    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;

    // if limelight can see tag set distance to tz
    boolean correctTag = false;
    if (LimelightHelpers.getTV(LimelightConstants.turretLimelight) && RobotBase.isReal()) {
        for (int tagID : MovementConstants.hubTagIDs) {
            // if not real ignore
            if (LimelightHelpers.getFiducialID(LimelightConstants.turretLimelight) == tagID) {
                correctTag = true;
                break;
            }
        }
        if (correctTag) {
            turretToTargetDistance = LimelightHelpers.getTargetPose_CameraSpace(LimelightConstants.turretLimelight)[2]; // distance is tz
        }
    }
    
    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }
    
    // Calculate parameters accounted for velocity
    turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    // if limelight use that angle
    if (correctTag) {
        double aprilTagOffset[] = LimelightHelpers.getTargetPose_CameraSpace(LimelightConstants.turretLimelight);
        turretToTargetDistance = aprilTagOffset[2]; // distance is tz
        // get angle
        turretAngle = new Rotation2d(Math.atan2(aprilTagOffset[2], aprilTagOffset[0]));
    }

    hoodAngle = launchHoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();
    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    turretVelocity =
        turretAngleFilter.calculate(
            turretAngle.minus(lastTurretAngle).getRadians() / Constants.deltaTime);
    hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.deltaTime);
    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;

    latestParameters =
        new LaunchingParameters(
            lookaheadTurretToTargetDistance >= minDistance
                && lookaheadTurretToTargetDistance <= maxDistance,
            turretAngle,
            turretVelocity,
            hoodAngle,
            hoodVelocity,
            launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));

    // Log calculated values
    // SmartDashboard.putNumber("Turret/LaunchCalculator/LookaheadPose", lookaheadPose);
    SmartDashboard.putNumber("Turret/LaunchCalculator/lookaheadTurretToTargetDistance", lookaheadTurretToTargetDistance);

    return latestParameters;
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }

 /**
   * Converts a Transform3d to a Transform2d
   *
   * @param transform The original transform
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(Transform3d transform) {
    return new Transform2d(
        transform.getTranslation().toTranslation2d(), transform.getRotation().toRotation2d());
  }
}