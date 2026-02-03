package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    public static final class DriveConstants {  
        // max speed MPS TODO: change max speed MPS
        public static final double maxSpeedMPS = 5;

         // starting position of bot
        public static final Pose2d startingPose = new Pose2d(new Translation2d(
            Meter.of(1),Meter.of(4)),Rotation2d.fromDegrees(0));
    }

    public static final class ControllerConstants {
        public static final double deadBand = 0.05;
    }
}