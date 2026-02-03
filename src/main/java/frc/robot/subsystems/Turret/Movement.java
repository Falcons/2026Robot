// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants.MovementConstants;
import frc.robot.subsystems.Swerve.Swerve;

public class Movement extends SubsystemBase {

  private final Swerve swerve;
  private final SparkMax pivot = new SparkMax(MovementConstants.pivotCANID, MotorType.kBrushless);
  private final SparkMaxConfig pivotConfig = new SparkMaxConfig();

  /** Creates a new Movement. */
  public Movement(Swerve swerve) {
    this.swerve = swerve;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void movePivot(double speed) {
    pivot.set(speed);
  }

  /**
   * @return the angle in which the shooter should be aiming at towards the goal in radians
   */
  public double getAimerAngle() {
    // Translation2d distanceToGoal = swerve.getPose().getTranslation().minus(AimerConstants.goalPos);
    Translation2d distanceToGoal = MovementConstants.goalPos.minus(swerve.getPose().getTranslation());
    // to get target angle use inverse tan O/A
    double targetAngle = Math.atan2(distanceToGoal.getY(), distanceToGoal.getX()); 
    
    return targetAngle;
  }
}
