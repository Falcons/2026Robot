// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Util.AllianceFlipUtil;
import frc.robot.subsystems.Turret.LaunchCalculator;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class Swerve extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */

  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive swerveDrive;

  public Swerve() {
    LimelightHelpers.SetIMUMode(LimelightConstants.stillLimelight, 0);
    try {
      // try to create a new swerve drive
      DriverStation.waitForDsConnection(0);
      // Pose2d limelightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.stillLimelight).pose;
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH; //TODO: high will cause more lag
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(DriveConstants.maxSpeedMPS, AllianceFlipUtil.apply(DriveConstants.startingPose));
      // swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(DriveConstants.maxSpeedMPS);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setMotorIdleMode(true);

    setupPathPlanner();
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyroWithFlip));  
  }

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();

    addVisionMeasurement(LimelightConstants.stillLimelight, true);
    addVisionMeasurement(LimelightConstants.turretLimelight, true);
    // addVisionMeasurement(LimelightConstants.stillLimelight, false);
    // addVisionMeasurement(LimelightConstants.turretLimelight, false);

  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }
  public void setPose(Pose2d pose){
    swerveDrive.resetOdometry(pose);
  }


  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.~
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        DriveConstants.maxSpeedMPS);
  }


  /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d speeds, Double rotation, Boolean fieldRelative){ 
    swerveDrive.drive(speeds, rotation, fieldRelative, false);  
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }


   /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Gets the max angular velocity of the robot
   *
   * @return the chassis max angular velocity
   */
  public double getMaximumAngularVelocity(){
    return swerveDrive.getMaximumChassisAngularVelocity();
  }
  /**
   * Gets the max velocity of the robot
   *
   * @return the chassis max velocity
   */
  public double getMaximumVelocity(){
    return swerveDrive.getMaximumChassisVelocity();
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    SmartDashboard.putNumber("desierd x", velocity.vxMetersPerSecond);
    SmartDashboard.putNumber("desierd y", velocity.vyMetersPerSecond);
    SmartDashboard.putNumber("desierd rot", velocity.omegaRadiansPerSecond);
    swerveDrive.driveFieldOriented(velocity);
  }
  public void zeroGyro(){
    swerveDrive.zeroGyro();
  }
  
  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0 (red alliance station).
   */
  public void zeroGyroWithFlip()
  {
    swerveDrive.swerveController.lastAngleScalar = 0;
    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
      swerveDrive.setGyro(new Rotation3d(0,0,Math.toRadians(180)));
      swerveDrive.setGyroOffset(swerveDrive.getGyroRotation3d());
      swerveDrive.resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d(Math.toRadians(180))));
    }else{
      swerveDrive.setGyro(new Rotation3d(0,0,0));
      swerveDrive.setGyroOffset(swerveDrive.getGyroRotation3d());
      swerveDrive.resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0 (red alliance station).
   */
  public void invertedZeroGyroWithFlip()
  {
    swerveDrive.swerveController.lastAngleScalar = 0;
    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
      swerveDrive.setGyro(new Rotation3d(0,0,0));
      swerveDrive.setGyroOffset(swerveDrive.getGyroRotation3d());
      swerveDrive.resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }else{
      swerveDrive.setGyro(new Rotation3d(0,0,Math.toRadians(180)));
      swerveDrive.setGyroOffset(swerveDrive.getGyroRotation3d());
      swerveDrive.resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d(Math.toRadians(180))));
    }
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          swerveDrive::getPose,
          // Robot pose supplier
          swerveDrive::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          swerveDrive::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(//built in path following controller for holonomic drive trains
              new PIDConstants(3, 0.0, 0.2), // Translation PID constants
              new PIDConstants(1.8, 0.0, 0.2)// Rotation PID constants
          ),
          
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

    /**
  * Adds the vision estimations from a limelight to swerveDriveodometry 
  * @param limelightNames name of limelight
  */
  public void addVisionMeasurement(String limelightName, boolean megatag2) {
    LimelightHelpers.SetRobotOrientation(limelightName, swerveDrive.getPose().getRotation().getDegrees(), 0, swerveDrive.getPitch().getDegrees(), 0, swerveDrive.getRoll().getDegrees(), 0);
    
    LimelightHelpers.PoseEstimate pose;
    if (megatag2) pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    else pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    boolean doRejectUpdate = false;
    // if our angular velocity is greater than 360 degrees per second, ignore vision updates
    if(Math.abs(swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond)) > 360)
    {
      doRejectUpdate = true;
    }
    if(pose != null){
      if(pose.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        swerveDrive.addVisionMeasurement(pose.pose, pose.timestampSeconds, VecBuilder.fill(.7,.7,9999999));
      }
    }
  }

  /**
   * sets the max alloweable speed of the swerve drive
   * @param velocity in meters per second
   * @param angularVelocity in radians per secnond
   */
  public void setMaxAllowableSpeed(double velocity, double angularVelocity) {
    swerveDrive.setMaximumAllowableSpeeds(velocity, angularVelocity);
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY){
    swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {
      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);
      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
        headingX.getAsDouble(),
        headingY.getAsDouble(),
        swerveDrive.getOdometryHeading().getRadians(),
        swerveDrive.getMaximumChassisVelocity()));
    });
  }

    /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param rotation     Angular velocity of the robot
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation){
    return run(() -> {

    Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),translationY.getAsDouble()), 0.8);
    Double rotationValue = rotation.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity();

    // Make the robot move
    drive(scaledInputs, rotationValue, true);
    });
  }
}