// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import com.revrobotics.PersistMode;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;



public class Climb extends SubsystemBase {

  private final SparkMax climb = new SparkMax(ClimbConstants.climbCANID, MotorType.kBrushless);
  private final SparkMaxConfig climbConfig = new SparkMaxConfig();
  
  // private final RelativeEncoder climbEncoder = climb.getEncoder();

  /** Creates a new Climb. */
  public Climb() {
    climb.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setClimb(double climbSpeed) {
    climb.set(climbSpeed);
  }

  public void stopClimb() {
    climb.stopMotor();
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
