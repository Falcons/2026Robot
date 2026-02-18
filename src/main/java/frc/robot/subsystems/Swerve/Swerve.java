// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class Swerve extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */

  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive swerveDrive;

  public Swerve() {

    try {
      // try to create a new swerve drive
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(DriveConstants.maxSpeedMPS, DriveConstants.startingPose);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    setupPathPlanner();
    setMaxSpeed(1.0, Math.toRadians(180));
  }

  @Override
  public void periodic() {
    addVisionMeasurement();
    SmartDashboard.putNumber("swerve/frontLeft encoder", swerveDrive.getModules()[0].getAbsolutePosition());
    SmartDashboard.putNumber("swerve/frontRight encoder", swerveDrive.getModules()[1].getAbsolutePosition());
    SmartDashboard.putNumber("swerve/backLeft encoder", swerveDrive.getModules()[2].getAbsolutePosition());
    SmartDashboard.putNumber("swerve/backRight encoder", swerveDrive.getModules()[3].getAbsolutePosition());
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void addVisionMeasurement(){
    Pose2d visionPose = LimelightHelpers.getBotPose2d_wpiBlue(DriveConstants.driveLimelight);
    swerveDrive.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
  }

  /**
   * 
   * @param translationalSpeed double, max movement speed in meters per second
   * @param angularSpeed double, max rotation speed in radians per second
   */
  public void setMaxSpeed(Double translationalSpeed, double angularSpeed){
    swerveDrive.setMaximumAllowableSpeeds(translationalSpeed, angularSpeed);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
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
   
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    SmartDashboard.putNumber("desierd x", velocity.vxMetersPerSecond);
    SmartDashboard.putNumber("desierd y", velocity.vyMetersPerSecond);
    SmartDashboard.putNumber("desierd rot", velocity.omegaRadiansPerSecond);
    swerveDrive.driveFieldOriented(velocity);
  } */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
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
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
        headingX.getAsDouble(),
        headingY.getAsDouble(),
        swerveDrive.getOdometryHeading().getRadians(),
        swerveDrive.getMaximumChassisVelocity()));
    });
  }
  
  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
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

    /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner(){
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try{
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
            if (enableFeedforward){
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else{
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // TODO: pid
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(4, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(3.13, 0.0, 0)
              // Rotation PID constants
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

    } catch (Exception e){
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }
}