// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret.TurretSim;


import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants.MovementConstants;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.subsystems.Turret.LaunchCalculator;

public class MovementSim extends SubsystemBase {

  private final Swerve swerve;
  private final Field2d field = new Field2d();
  
  private final Servo leftHoodActuatorSim = new Servo(MovementConstants.leftHoodActuatorPWM);
  private final Servo rightHoodActuatorSim = new Servo(MovementConstants.rightHoodActuatorPWM);

  // simulated motor
  private final SparkMax turret = new SparkMax(MovementConstants.turretCANID, MotorType.kBrushless);
  private final SparkMaxSim turretSim = new SparkMaxSim(turret, DCMotor.getNEO(1));

  // sim encoder
  private final SparkAbsoluteEncoderSim turretEncoderSim = turretSim.getAbsoluteEncoderSim();

  private final PIDController turretPID = new PIDController(0.05, 0, 0);

  private boolean atMax = false, atMin = false;

  // simulated variables
  private Rotation2d turretDir = new Rotation2d();
  private Pose2d turretPose = new Pose2d();

  /** Creates a new Movement. */
  public MovementSim(Swerve swerve) {
    // leftHoodActuator.createDouble("position", Direction.kBidir, 0);
    // rightHoodActuator.createDouble("position", Direction.kBidir, 0);
    this.swerve = swerve;
    SmartDashboard.putData("Field", field);
    // turretPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // check if max or min
    atMax = turretEncoderSim.getPosition() >= MovementConstants.turretMaxRad;
    atMin = turretEncoderSim.getPosition() <= MovementConstants.turretMinRad;

    // for sim make a direction and a pose, add robot radians becuase turret rotates with bot
    turretDir = new Rotation2d(turretEncoderSim.getPosition() + swerve.getPose().getRotation().getRadians());
    turretPose = new Pose2d(swerve.getPose().getTranslation(), turretDir);
    field.getObject("turret").setPose(turretPose);

    SmartDashboard.putBoolean("Turret/Movment/Turret/at max", atMax);
    SmartDashboard.putBoolean("Turret/Movment/Turret/at min", atMin);
    SmartDashboard.putNumber("Turret/Movememnt/Hood/left actuators", leftHoodActuatorSim.getAngle());
    SmartDashboard.putNumber("Turret/Movememnt/Hood/right actuators", rightHoodActuatorSim.getAngle());
    SmartDashboard.putNumber("Turret/MovementSim/globalAngle", Math.toDegrees(getGlobalRad()));
    SmartDashboard.putNumber("Turret/MovementSim/relativeAngle", Math.toDegrees(getRelativeRad()));
  }

  /**
   * call this to actually auto aim to the goal
   */
  public void autoAim() {
    // clamp setpoint
    double setpoint = MathUtil.clamp(getRelativeRad(), MovementConstants.turretMinRad, MovementConstants.turretMaxRad);
    
    // calc pid
    double pid = turretPID.calculate(turret.getAbsoluteEncoder().getPosition(), setpoint);
    // clamp pid 
    pid = MathUtil.clamp(pid, -0.5, 0.5);

    // move motor
    setTurret(pid);
    turretPID.reset();
  }

  /**
   * move the pivot wiht a clamp
   * @param speed the speed to move the turret
   */
  public void setTurret(double speed) { 
    // clamp
    if (atMax && speed > 0) {
      speed = 0; turret.stopMotor();
    } 
    if (atMin && speed < 0) {
      speed = 0; turret.stopMotor();
    }
    turretSim.setAppliedOutput(speed);
    turretEncoderSim.setPosition(turretEncoderSim.getPosition() + speed);
  }
  public void aimHood(){
    double setpoint = MathUtil.clamp(getHoodAngle(), 0, 1);
    setHood(setpoint);
  }

  /**
   * @return the angle in which the shooter should be aiming at towards the goal in radians -pi to pi
   */
  public double getGlobalRad() {
    // // Translation2d distanceToGoal = swerve.getPose().getTranslation().minus(AimerConstants.goalPos);
    // Translation2d distanceToGoal = MovementConstants.goalPos.minus(swerve.getPose().getTranslation());
    // // to get target angle use inverse tan O/A
    // double targetAngle = Math.atan2(distanceToGoal.getY(), distanceToGoal.getX()); 
    
    // return MathUtil.angleModulus(targetAngle);

    // use the launch calulator to get global angle
    LaunchCalculator.getInstance().clearLaunchingParameters();
    return LaunchCalculator.getInstance().getParameters(swerve).turretAngle().getRadians();
  }
  /**
   * @return the angle in which the hood should be aiming at 
   */
  public double getHoodAngle() {
    // use the launch calulator to get hood angle
    LaunchCalculator.getInstance().clearLaunchingParameters();
    return LaunchCalculator.getInstance().getParameters(swerve).hoodAngle();
  }

  /**
   * @return the relative abgle the shooter should point at in radians -pi to pi
   */
  public double getRelativeRad() {
    return MathUtil.angleModulus(getGlobalRad() - swerve.getHeading().getRadians());
  }

  /**
   * Set the hood to the position
   * @param Position position in degrees
   */
  public void setHood(double Position){
    leftHoodActuatorSim.set(Position);
    rightHoodActuatorSim.set(Position);
  }

  /**
   * move hood based on commanded position + speed in degrees
   * @param speed degrees * 20  /second
   */
  public void moveHood(double speed){
    leftHoodActuatorSim.setAngle(leftHoodActuatorSim.getAngle() + speed);
    leftHoodActuatorSim.setAngle(leftHoodActuatorSim.getAngle() + speed);
  }

  /**
   * @return true if turret is in range
   */
  public boolean turretInRange() {
    return getGlobalRad() - getRelativeRad() < MovementConstants.turretError;
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
    return turretInRange() && hoodInRange();
  }
}
