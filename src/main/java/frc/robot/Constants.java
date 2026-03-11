package frc.robot;

import static edu.wpi.first.units.Units.Meter;

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
    public static final boolean isCompetition = false;

    public static final class DriveConstants {  
        // max speed MPS TODO: change max speed MPS
        public static final double maxSpeedMPS = 6.7;
        public static final double maxRotationsRadPS = Math.toRadians(1080);
        public static final double slowModeMPS = 1;
        public static final double slowModeRotationsRadPS = Math.toRadians(45);
        
        public static final double driveRatio = 6.75;
        public static final double driveSteeringRatio = 150/7;

        public static final PathConstraints pathFindingConstraints = new PathConstraints(5, 5, 540, 270);

         // starting position of bot
        public static final Pose2d startingPose = new Pose2d(new Translation2d(
            Meter.of(3),//X
            Meter.of(4)), //Y
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
    }

    public static final class ControllerConstants {
        public static final double deadBand = 0.05;
        public static final double triggerDeadBand = 0.01;
    }

    public static final class HoodConstants {
        public static final double hoodDefault = 0;

        public static final int rightHoodActuatorPWM = 0;
        public static final int leftHoodActuatorPWM = 1;

        public static final int hoodAngleMin = 18;
        public static final int hoodAngleMax = 126;

        // 35:1 gear ratio so 32mm per second and 100mm legnth
        // 32/100 is our max speed so multiply that by joystick -1 - 1 will give speed\
        // however there is load soooo divide by random number lol //TODO: speed multiplier
        public static final double hoodSpeedMultipier = 0.32 / 5;
    }

    public static final class TurretConstants {

        public static final int turretCANID = 14;
            
        public static final double turretMinRad = Math.toRadians(10); 
        public static final double turretMidRad = Math.toRadians(90);
        public static final double turretMaxRad = Math.toRadians(160);

        public static final double turretError = Math.toRadians(3); //TODO: turret error

        // shoot on the move
        public static Transform3d robotToTurret = new Transform3d(0.114, 0, 0.30226, Rotation3d.kZero);
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
        }
    }

    public static final class IntakeConstants {

        public static final class RollersConstants {
            public static final int rollerCANID = 6;
            public static final double rollerSpeed = 1;
        }

        public static final class PivotConstants {
            public static final int pivotCANID = 22;

            public static final double pivotMin = Math.toRadians(207);
            public static final double pivotOut = Math.toRadians(220);
            public static final double pivotIn = Math.toRadians(322); // in is higher, out is lower
            public static final double pivotMax = Math.toRadians(325); // in is higher, out is lower
        }
    }
}