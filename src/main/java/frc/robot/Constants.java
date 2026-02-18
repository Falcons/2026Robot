package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    public static final class DriveConstants {  
        // max speed MPS TODO: change max speed MPS
        public static final double maxSpeedMPS = 6.7;
        
        public static final double driveRatio = 6.75;
        public static final double driveSteeringRatio = 150/7;
        public static final String driveLimelight = "limelight-tag";

         // starting position of bot
        public static final Pose2d startingPose = new Pose2d(new Translation2d(
            Meter.of(3),Meter.of(4)),Rotation2d.fromDegrees(0));
    }

    public static final class ControllerConstants {
        public static final double deadBand = 0.05;
    }
}