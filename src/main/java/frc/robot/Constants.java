package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final double deltaTime = 0.02;
    public static final boolean disableHAL = false; // if tue alliance doesnt update in sim
    public static final boolean isCompetition = false;

    public static final class DriveConstants {  
        // max speed MPS TODO: change max speed MPS
        public static final double maxSpeedMPS = 6.7;
        public static final double slowModeMPS = 1;
        public static final double slowModeRPS = Math.toRadians(45);
        
        public static final double driveRatio = 6.75;
        public static final double driveSteeringRatio = 150/7;

        public static final PathConstraints pathFindingConstraints = new PathConstraints(5, 5, 540, 270);

         // starting position of bot
        public static final Pose2d startingPose = new Pose2d(new Translation2d(
            Meter.of(3),//X
            Meter.of(4)), //Y
            Rotation2d.fromDegrees(180));
        

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
        public static final String turretLimelight = "limelight-turret"; // TODO: limelight names

        // TODO: limelight pos
        public static final Double yOffset = 0.0; // the vertical offset (from centre of the bot to the centre of the circle)
        public static final Double xOffset = 0.0; // the horizontal offset (from centre of the bot to the centre of the circle)
        public static final Double radius = 0.0; // the radius of the limelight circle

        public static final Double up = 0.0; // up Up offset in meters
        public static final Double roll = 0.0; // roll Roll angle in degrees
        public static final Double pitch = 0.0 ;// pitch Pitch angle in degrees
       public static final Double yaw = 0.0; // yaw Yaw angle in degrees
    }

    public static final class ControllerConstants {
        public static final double deadBand = 0.05;
        public static final double triggerDeadBand = 0.01;
    }

    public static final class TurretConstants {

        public static final class MovementConstants {

            public static final int turretRatio = 48; //48 motor rotations per 1 turret rotation
            public static final int hoodRatio = 100;

            public static final double hoodDefault = 0;
            public static final double hoodDownDistanceMaxM = 6; //TODO: change hood distances i think the middle is 8m?
            public static final double hoodDownDistanceMinM = 10;

            public static final int turretCANID = 14; // TODO: canids
            public static final int HoodActuatorPWM = 0;
            
            public static final Translation2d goalPos = new Translation2d(3, 5); // TODO: goal pos and turret range
            public static final double turretMinRad = Math.toRadians(-180); // TODO: change min and max
            public static final double turretMaxRad = Math.toRadians(180);

            public static final double turretError = Math.toRadians(3); //TODO: turret error

            public static final int hubTagIDs[] = {0};
        }
        public static final class ShooterConstants {

            public static final double kickerRatio = 4;
            public static final int transferRatio = 4;

            public static final int leftShooterCANID = 21;
            public static final int rightShooterCANID = 23;
            public static final int transferCANID = 20;
            public static final int kickerCANID = 24; 

            public static final double maxShooterRPS = 80;
            public static final double maxShooterSpeed = 1;
            public static final double maxTransferSpeed = 1;
            public static final double maxKickerSpeed = 1;
        }

        // shoot on the move
        public static Transform3d robotToTurret = new Transform3d(0, 0.0, 0, Rotation3d.kZero); //TODO: change these
        public static Transform3d turretToCamera =
        new Transform3d(
            0, 0.0, 0, new Rotation3d(0.0, Units.degreesToRadians(0), 0.0));
    }

    public static final class IntakeConstants {

        public static final class RollersConstants {
            public static final double intakeRollersRatio = 3.2;
            public static final int rollerCANID = 6;
            public static final double rollerSpeed = 1;
        }

        public static final class PivotConstants {
            public static final int pivotCANID = 22;

            public static final double pivotMin = Math.toRadians(207); // TODO: pivot setpoints
            public static final double pivotOut = Math.toRadians(220); // TODO: pivot setpoints
            public static final double pivotShake = Math.toRadians(45);
            public static final double pivotIn = Math.toRadians(300); // in is higher, out is lower
            public static final double pivotMax = Math.toRadians(305); // in is higher, out is lower

            public static final int intakePivotRatio = 48; // 48 motor rotations per 1 pivot rotation
        }
    }
}