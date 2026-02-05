// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;


public class Climb extends SubsystemBase {

  private final SparkMax climb = new SparkMax(ClimbConstants.climbID, MotorType.kBrushless);
  
  private final RelativeEncoder liftEncoder = climb.getEncoder();

  

  /** Creates a new Climb. */
  public Climb() 
  {
  
  }

  public void setClimb(double climbSpeed)
  {
    climb.set(climbSpeed);
  
  }

  public void setLeftClimb(double speed)
  {
    climb.set(speed);
  }

  public void setBrakeMode()
  {
    climb.setIdleMode(IdleMode.kBrake);
  }

  public void stopClimb() 
  {
    climb.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
