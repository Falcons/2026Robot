package frc.robot;

import static edu.wpi.first.units.Units.Meter;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    public static final double loopPeriodSecs = 0.02; // delta time

    public static final class DriveConstants {  
        // max speed MPS TODO: change max speed MPS
        public static final double maxSpeedMPS = 5;
        
        public static final double driveRatio = 6.75;
        public static final double driveSteeringRatio = 150/7;

         // starting position of bot
        public static final Pose2d startingPose = new Pose2d(new Translation2d(
            Meter.of(3),Meter.of(4)),Rotation2d.fromDegrees(-90));
    }

    public static final class LimelightConstants {
        public static final String turretLimelight = "limelight-turret";
    }

    public static final class ControllerConstants {
        public static final double deadBand = 0.05;
    }

    public static final class TurretConstants {

        public static final class MovementConstants {

            public static final int turretRatio = 48;
            public static final int hoodRatio = 100;

            public static final int turretCANID = 1000; // TODO: canid
            public static final int leftHoodActuatorPWM = 1006; //TODO: set PWM channel
            public static final int rightHoodActuatorPWM = 1007; //TODO: set PWM channel
            
            public static final Translation2d goalPos = new Translation2d(3, 5); // TODO: goal pos and turret range
            public static final double turretMinRad = Math.toRadians(-80); // TODO: change min and max
            public static final double turretMaxRad = Math.toRadians(80);

            public static final double turretError = Math.toRadians(3); //TODO: turret error

            public static final int hubTagIDs[] = {0};
        }
        public static final class ShooterConstants {

            public static final double kickerRatio = 4;
            public static final int transferRatio = 4;

            public static final int leftShooterCANID = 1001; // TODO: canid
            public static final int rightShooterCANID = 1002; // TODO: canid
            public static final int transferCANID = 1004; // TODO: canid
            public static final int kickerCANID = 1005; // TODO: canid

            public static final double maxShooterSpeed = 300;
            
        }
    }

    public static final class IntakeConstants {

        public static final class RollersConstants {
            public static final double intakeRollersRatio = 3.2;
            public static final int rollerCANID = 8087; // TODO: canid
            public static final double rollerSpeed = 1;
        }

        public static final class PivotConstants {
            public static final int pivotCANID = 1050; // TODO: canid

            public static final double pivotOut = Math.toRadians(0); // TODO: pivot setpoints
            public static final double pivotShake = Math.toRadians(45);
            public static final double pivotIn = Math.toRadians(90); // in is higher, out is lower

            public static final int intakePivotRatio = 48;
            public static final int singlizerRatio = 4;
        }
    }

    public static final class ClimbConstants {
        public static final int climbCANID = 1432; //TODO: canid
    }
}