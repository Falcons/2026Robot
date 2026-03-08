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
import edu.wpi.first.math.interpolation.Interpolator;
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
      double hoodVelocity) {}

  // Cache parameters
  private LaunchingParameters latestParameters = null;

  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;

  // < distance : < rps : hood angle > >
  private static final InterpolatingTreeMap<Double, InterpolatingTreeMap<Double, Double>> hoodAngleMap =
    // a (input), b (input), t (interpolation fraction) return a -> 
    //doesnt interpolate because idk how to intrepertate an interpretation map ._.
    new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), (a, b, t) -> a); 
  // 3m
  private static final InterpolatingTreeMap<Double, Double> speed3m = 
    new InterpolatingTreeMap<Double, Double>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  // 4m
  private static final InterpolatingTreeMap<Double, Double> speed4m = 
    new InterpolatingTreeMap<Double, Double>(InverseInterpolator.forDouble(), Interpolator.forDouble());
      
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 1.34;
    maxDistance = 5.6;
    phaseDelay = 0.03;
    
    // 3m
    speed3m.put(80.0, 20.0); // RPM to hood deg
    speed3m.put(90.0, 25.0);
    speed3m.put(100.0, 30.0); 
    hoodAngleMap.put(3.0, speed3m);

    // 4m
    speed4m.put(80.0, 25.0);
    speed4m.put(90.0, 28.0);
    speed4m.put(100.0, 32.0);
    hoodAngleMap.put(4.0, speed4m);


    // Existing entries
    timeOfFlightMap.put(1.38, 0.90);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(5.68, 1.16);
  }

  public LaunchingParameters getParameters(Swerve swerve, Double shooterSpeed) {
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
    if (LimelightHelpers.lookingAtHub(LimelightConstants.turretLimelight)) {
        correctTag = true;
        turretToTargetDistance = LimelightHelpers.getTargetPose_CameraSpace(LimelightConstants.turretLimelight)[2]; // distance is tz
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

    // get hood angle from maps
    // use distance to get all speed maps and than use shooter speed to get hood angle
    hoodAngle = hoodAngleMap.get(lookaheadTurretToTargetDistance).get(shooterSpeed);

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
            hoodVelocity);

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