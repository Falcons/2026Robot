// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.Constants.TurretConstants.MovementConstants;
import frc.robot.subsystems.Swerve.Swerve;

public class Movement extends SubsystemBase {

  private final Swerve swerve;

  private final SparkMax pivot = new SparkMax(MovementConstants.turretCANID, MotorType.kBrushless);
  private final AbsoluteEncoder pivotEncoder = pivot.getAbsoluteEncoder();
  private final SparkMaxConfig pivotConfig = new SparkMaxConfig();

  private final Servo leftHoodActuators = new Servo(MovementConstants.leftHoodActuatorPWM);
  private final Servo rightHoodActuators = new Servo(MovementConstants.rightHoodActuatorPWM);

  private final PIDController pivotPID = new PIDController(0.05, 0, 0);

  private boolean atMax, atMin;

  /** Creates a new Movement. */
  public Movement(Swerve swerve) {
    this.swerve = swerve;

    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret/Movmement/Pivot/Speed", pivot.get());
    SmartDashboard.putNumber("Turret/Movmement/Pivot/Absolute Encoder", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Turret/Movememnt/Hood/left actuators", leftHoodActuators.get());
    SmartDashboard.putNumber("Turret/Movememnt/Hood/right actuators", rightHoodActuators.get());

    // check if max or min
    atMax = pivot.get() >= MovementConstants.turretMin;
    atMin = pivot.get() <= MovementConstants.turretMax;
  }

  //TODO: add turn direction to comment
  /**
   * @param speed speed to turn the pivot
   */
  public void setPivot(double speed) {
    // make sure the pivot is in range before rotating
    if (pivotEncoder.getPosition() > MovementConstants.turretMin && pivotEncoder.getPosition() < MovementConstants.turretMax){
      pivot.set(speed);
    }
  }

  /**
   * call this to actually auto aim to the goal
   */
  public void autoAim() {
    // clamp speed
    double setpoint = MathUtil.clamp(getRelativeAngle(), PivotConstants.pivotMin, PivotConstants.pivotMax);
    // calc pid
    double pid = pivotPID.calculate(pivot.getAbsoluteEncoder().getPosition(), setpoint);
    // clamp setpoint 
    pid = MathUtil.clamp(pid, -0.5, 0.5);

    // move motor
    setPivotPID(pid);
    pivotPID.reset();
  }

  /*
   * run pivot pid with a clamp
   */
  public void setPivotPID(double speed) { 
    // clamp
    if (atMax && speed > 0) {
      speed = 0; pivot.stopMotor();
    } 
    if (atMin && speed < 0) {
      speed = 0; pivot.stopMotor();
    }
    setPivot(speed);
  }

  public void moveHood(){
    
  }

  /**
   * @return the angle in which the shooter should be aiming at towards the goal in radians
   */
  public double getGlobalAngle() {
    // Translation2d distanceToGoal = swerve.getPose().getTranslation().minus(AimerConstants.goalPos);
    Translation2d distanceToGoal = MovementConstants.goalPos.minus(swerve.getPose().getTranslation());
    // to get target angle use inverse tan O/A
    double targetAngle = Math.atan2(distanceToGoal.getY(), distanceToGoal.getX()); 
    
    return targetAngle;
  }

  /**
   * @return the relative abgle the shooter should point at in radians
   */
  public double getRelativeAngle() {
    return swerve.getHeading().getRadians() - getGlobalAngle();
  }

  /**
   * @return true if pivot is in range
   */
  public boolean pivotInRange() {
    return getGlobalAngle() - getRelativeAngle() < MovementConstants.turretError;
  }

  /**
   * @return true if hood is in range
   */
  public boolean hoodInRange() {
    return false;
  }

  /**
   * checks if the turret is in range
   * @return a bool, true if in range
   */
  public boolean inRange() {
    return pivotInRange() && hoodInRange();
  }
}
