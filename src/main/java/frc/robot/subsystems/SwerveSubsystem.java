// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */

  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive  swerveDrive;

  public SwerveSubsystem() {

    try {
      // try to create a new swerve drive
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(DriveConstants.maxSpeedMPS, DriveConstants.startingPose);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("swerve/frontLeft encoder", swerveDrive.getModules()[0].getAbsolutePosition());
    SmartDashboard.putNumber("swerve/frontRight encoder", swerveDrive.getModules()[1].getAbsolutePosition());
    SmartDashboard.putNumber("swerve/backLeft encoder", swerveDrive.getModules()[2].getAbsolutePosition());
    SmartDashboard.putNumber("swerve/backRight encoder", swerveDrive.getModules()[3].getAbsolutePosition());
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
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
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    SmartDashboard.putNumber("desierd x", velocity.vxMetersPerSecond);
    SmartDashboard.putNumber("desierd y", velocity.vyMetersPerSecond);
    SmartDashboard.putNumber("desierd rot", velocity.omegaRadiansPerSecond);
    swerveDrive.driveFieldOriented(velocity);
  }
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