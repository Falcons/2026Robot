// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;


import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants.MovementConstants;
import frc.robot.subsystems.Swerve.Swerve;

public class TurretSim extends SubsystemBase {

  private final Swerve swerve;
  private final Field2d field = new Field2d();

  // simulated motor
  private final SparkMax turret = new SparkMax(MovementConstants.turretCANID-200, MotorType.kBrushless);
  private final SparkMaxSim turretSim = new SparkMaxSim(turret, DCMotor.getNEO(1));

  // sim encoder
  private final SparkAbsoluteEncoderSim turretEncoderSim = turretSim.getAbsoluteEncoderSim();

  private final PIDController turretPID = new PIDController(0.05, 0, 0);

  private boolean atMax = false, atMin = false;

  // simulated variables
  // turret
  private Pose2d turretPose = new Pose2d();

  /** Creates a new Movement. */
  public TurretSim(Swerve swerve) {
    this.swerve = swerve;
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    // check if max or min
    atMax = turretEncoderSim.getPosition() >= MovementConstants.turretMaxRad;
    atMin = turretEncoderSim.getPosition() <= MovementConstants.turretMinRad;

    // for sim make a direction and a pose, add robot radians becuase turret rotates with bot
    turretPose = new Pose2d(swerve.getPose().getTranslation(), 
                 new Rotation2d(turretEncoderSim.getPosition() + swerve.getPose().getRotation().getRadians()));

    field.getObject("turret").setPose(turretPose);

    SmartDashboard.putBoolean("Turret/TurretSim/at max", atMax);
    SmartDashboard.putBoolean("Turret/TurretSim/at min", atMin);
    SmartDashboard.putNumber("Turret/TurretSim/globalAngle", Math.toDegrees(getGlobalRad()));
    SmartDashboard.putNumber("Turret/TurretSim/relativeAngle", Math.toDegrees(getRelativeRad()));
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
    set(pid);
    turretPID.reset();
  }

  /**
   * move the pivot wiht a clamp
   * @param speed the speed to move the turret
   */
  public void set(double speed) { 
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

  /**
   * @return the angle in which the shooter should be aiming at towards the goal in radians -pi to pi
   */
  public double getGlobalRad() {
    LaunchCalculator.getInstance().clearLaunchingParameters();
    return LaunchCalculator.getInstance().getParameters(swerve).turretAngle().getRadians();
  }

  /**
   * @return the relative abgle the shooter should point at in radians -pi to pi
   */
  public double getRelativeRad() {
    return MathUtil.angleModulus(getGlobalRad() - swerve.getHeading().getRadians());
  }

  /**
   * @return true if turret is in range
   */
  public boolean inRange() {
    return getGlobalRad() - getRelativeRad() < MovementConstants.turretError;
  }
}
