package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import static java.util.Map.entry;
import java.util.Map;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final double deltaTime = 0.02;
    public static final boolean disableHAL = false; // if tue alliance doesnt update in sim
    public static final boolean isCompetition = true;
    public static final boolean logDashboard = true;

    public static final class DriveConstants {  
        // max speed MPS TODO: change max speed MPS
        public static final double maxSpeedMPS = 6.7;
        public static final double maxRotationsRadPS = Math.toRadians(1080);
        public static final double slowModeMPS = 2;
        public static final double slowModeRotationsRadPS = Math.toRadians(200);
        
        public static final double driveRatio = 6.75;
        public static final double driveSteeringRatio = 150/7;

        public static final PathConstraints pathFindingConstraints = new PathConstraints(6.7, 5, 6.8, 4);

         // starting position of bot
        public static final Pose2d startingPose = new Pose2d(new Translation2d(
            Meter.of(3.5),//X 3,5
            Meter.of(4)), //Y
            Rotation2d.fromDegrees(0));
         // starting position of bot
        public static final Pose2d startingPoseLeftBump = new Pose2d(new Translation2d(
            Meter.of(3.5),//X 3,5
            Meter.of(5.7)), //Y
            Rotation2d.fromDegrees(0));
         // starting position of bot
        public static final Pose2d startingPoseRightBump = new Pose2d(new Translation2d(
            Meter.of(3.5),//X 3,5
            Meter.of(2.7)), //Y
            Rotation2d.fromDegrees(0));

        // corrner hideing pose aka timeout
        public static final Pose2d timeoutPoseLeft = new Pose2d(new Translation2d(
                Meter.of(0.7), //X
                Meter.of(7.3)), //Y
                Rotation2d.fromDegrees(0));

        // corrner hideing pose aka timeout
        public static final Pose2d timeoutPoseRight = 
            new Pose2d(new Translation2d(
                Meter.of(0.700), //X
                Meter.of(2)), //Y
                Rotation2d.fromDegrees(0));

        public static final Pose2d ShootingStartLeft = new Pose2d(new Translation2d(
            Meter.of(1.7), //X
            Meter.of(5)), //Y
            Rotation2d.fromDegrees(0));
        
        public static final Pose2d ShootingStartRight = new Pose2d(new Translation2d(
            Meter.of(1.7), //X
            Meter.of(2)), //Y
            Rotation2d.fromDegrees(0));
    }

    public static final class LimelightConstants {
        public static final String turretLimelight = "limelight-turret";
        public static final String stillLimelight = "limelight-still";
        public static final Translation3d turretLimelightPos = new Translation3d(0.303022, 0, 0.30226);
        public static final Translation3d stillLimelightPos = new Translation3d(Units.inchesToMeters(7.76), Units.inchesToMeters(8.93), Units.inchesToMeters(10.135));
        public static final int blueHubTagIDs[] = {18, 19, 29, 21, 24, 25, 26, 27};
        public static final int redHubTagIDs[] = {2, 3, 4, 5, 8, 9, 10, 11};

        public static final Rotation3d turretLimelightRot = new Rotation3d(0, Math.toRadians(26), 0);

        // public static final Map<Double, Transform2d> tagOffsets = new Map()
            
        public static final Map<Double, Translation3d> tagOffsets = Map.ofEntries(
            entry(-1.0, new Translation3d(0.0, 0.0, 0.0)),
            entry(2.0, new Translation3d(-0.584, 0.0, 0.0)),
            entry(3.0, new Translation3d(0.0, 0.0, 0.0)),
            entry(4.0, new Translation3d(0.0, 0.0, 0.0)),
            entry(5.0, new Translation3d(0.0, 0.0, 0.0)),
            entry(8.0, new Translation3d(0.0, 0.0, 0.0)),
            entry(9.0, new Translation3d(0.1, 0.584, 0.0)),
            entry(10.0, new Translation3d(0.0, 1, 0.0)),
            entry(11.0, new Translation3d(0.0, 0.0, 0.0))
            
        );
        // public Dictionary<String, Double> tagOffsets = new Hashtable<>();
    }
    public static final class ControllerConstants {
        public static final double deadBand = 0.05;
        public static final double turretDeadband = 0.25;
        public static final double triggerDeadBand = 0.01;
    }

    public static final class HoodConstants {
        public static final double hoodDefault = 0.1;

        public static final int rightHoodActuatorPWM = 0;
        public static final int leftHoodActuatorPWM = 1;

        public static final int hoodAngleMin = 0;
        public static final int hoodAngleMax = 126;

        public static final double hoodMin = 0.1;
        public static final double hoodMax = 0.7;

        public static final double hoodSpeedMultipier = 0.1;
    }

    public static final class TurretConstants {

        public static final int turretCANID = 14;
            
        public static final double turretMinRad = Math.toRadians(6); 
        public static final double turretMidRad = Math.toRadians(91);
        public static final double turretMaxRad = Math.toRadians(171);

        public static final double turretError = Math.toRadians(3);

        // shoot on the move
        public static Transform3d robotToTurret = new Transform3d(0.114, 0, 0.30226, Rotation3d.kZero);
        public static Transform3d robotToTurretZero = new Transform3d(0, 0, 0, Rotation3d.kZero);
        // x: 4.5in
        // y: 15in - using 11.9 cuz limelight
        public static Transform3d turretToCamera =
            new Transform3d(0, 0.0, 0, new Rotation3d(0.0, Units.degreesToRadians(0), 0.0));
        
        public static final class ShooterConstants {

            public static final int leftShooterCANID = 21;
            public static final int rightShooterCANID = 23;
            public static final int transferCANID = 20;
            public static final int kickerCANID = 24; 

            public static final double maxShooterRPS = 80;
            public static final double maxShooterSpeed = 1;
            public static final double maxTransferSpeed = 1;
            public static final double maxKickerSpeed = 1;

            public static final double highShooterSpeed = 0.8;
            public static final double highShooterHoodAngle = 0.1;
            public static final double lowShooterSpeed = 0.6;
            public static final double lowShooterHoodAngle = 0.8;

            public static final double shooterErrorRps = 3; 
        }
    }

    public static final class IntakeConstants {

        public static final class RollersConstants {
            public static final int rollerCANID = 6;
            public static final double rollerSpeed = 1;
            public static final double rollerSpeedRPS = 105;
            public static final double slowRollerSpeed = 0.7;
            public static final double slowRollerSpeedRPS = 30;
        }

        public static final class PivotConstants {
            public static final int pivotCANID = 22;

            public static final double pivotMin = Math.toRadians(199); // in is higher, out is lower
            public static final double pivotMax = Math.toRadians(330); // in is higher, out is lower

             public static final double pivotOut = Math.toRadians(199);
            public static final double pivotIn = Math.toRadians(326);
        }
    }
    public static final class LightConstants {
        public static final int lightPwm = 2;
        public static final double resetInterval = 1;

        public static final double hoodUp = 0.61;
        public static final double hoodDown = 0.77;

        public static final int autoFirePriority = 0;
        public static final int autoFireAimPriority = 1;
        public static final int manualTransferPriority = 2;
        public static final int manualShooterPriority = 3;
        public static final int hoodUpPriority = 4;
        public static final int intakePriority = 5;
        public static final int hoodDownPriority = 6;
    }
}