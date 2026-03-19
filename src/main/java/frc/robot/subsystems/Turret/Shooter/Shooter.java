// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret.Shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
// import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.LightConstants;
import frc.robot.Constants.TurretConstants.ShooterConstants;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.subsystems.Turret.LaunchCalculator;
import frc.robot.subsystems.Turret.Turret;

public class Shooter extends SubsystemBase {
  
  // Orchestra orchestra = new Orchestra();///music bs
  private final Lights lights;

  // once shooters AND kicker are max speed than transfer
  private final TalonFX leftShooter = new TalonFX(ShooterConstants.leftShooterCANID); // Kraken x60
  private final TalonFX rightShooter = new TalonFX(ShooterConstants.rightShooterCANID);
  private final TalonFX kicker = new TalonFX(ShooterConstants.kickerCANID);

  // configs
  private final TalonFXConfiguration leftShooterConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration rightShooterConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration kickerConfig = new TalonFXConfiguration();

  private final CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs();

  // need to connect to movement to see if its aligned
  private final Turret aimer;
  private final Swerve swerve;

  // variables
  public boolean shooterRunning = false;
  private double targetRps = 0;

  // pid
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final MotionMagicVelocityVoltage motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0);
  private final PIDController speedControl = new PIDController(0.46, 0, 0.05);
  // sys iddd
  private final SysIdRoutine sysIdRoutine;
  private final MutVoltage mutVoltage = Volts.mutable(0);
  private final MutAngularVelocity mutAngularVelocity = RotationsPerSecond.mutable(0);

  /** Creates a new Shooter. */
  public Shooter(Turret aimer, Swerve swerve, Lights lights) {

    // sys iddddd
    // https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/sysidroutine/subsystems/Shooter.java
    // https://github.com/Spectrum3847/2026-Spectrum/blob/main/src/main/java/frc/robot/swerve/SysID.java
    sysIdRoutine = new SysIdRoutine( // should run for 10 seconds
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          (volts) -> {
              leftShooter.setVoltage(volts.in(Volts)); //TODO: will follow?
              // rightShooter.setVoltage(volts.in(Volts));
          },
          // ill look into ctre signal logger one day
          // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/api-usage/signal-logging.html
          // https://v6.docs.ctr-electronics.com/en/latest/docs/tuner/tools/log-extractor.html
          log -> {
            log.motor("shooter")
                .voltage(mutVoltage.mut_replace(
                    leftShooter.getMotorVoltage().getValueAsDouble(), Volts))
                .angularVelocity(mutAngularVelocity.mut_replace(
                    leftShooter.getVelocity().getValueAsDouble(), RotationsPerSecond));
          }, // download logs https://docs.wpilib.org/en/stable/docs/software/driverstation/driver-station-log-viewer.html
          // using the gear icon do upload wpilogs
          // nvm you are probably going to need this:
          // https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog-download.html
          this
      )
    );
    // analysis https://docs.frcteam3636.com/sysid 
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/loading-data.html
    // screew this jus tlike do it manalluy
    // https://phoenixpro-documentation--161.org.readthedocs.build/en/161/docs/application-notes/manual-pid-tuning.html
    // https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/closed-loop-requests.html#static-feedforward-sign

    // reset to factor defaults
    leftShooter.getConfigurator().apply(new TalonFXConfiguration());
    rightShooter.getConfigurator().apply(new TalonFXConfiguration());
    kicker.getConfigurator().apply(new TalonFXConfiguration());
  
    this.aimer = aimer;
    this.swerve = swerve;
    this.lights = lights;

    rightShooter.getConfigurator().apply(new TalonFXConfiguration());
    leftShooter.getConfigurator().apply(new TalonFXConfiguration());
    

    // smart current limits
    limitsConfigs.StatorCurrentLimit = 40;
    limitsConfigs.StatorCurrentLimitEnable = true;

    // coast
    leftShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // pidddddddd
    var slot0Configs = new Slot0Configs(); // shouldnt need a
    slot0Configs.kS = 0.4; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.11; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = 0; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output

    // motion magic to make more smooth
    leftShooterConfig.MotionMagic.MotionMagicAcceleration = 400;
    leftShooterConfig.MotionMagic.MotionMagicJerk = 4000;

    rightShooterConfig.MotionMagic.MotionMagicAcceleration = 400;
    rightShooterConfig.MotionMagic.MotionMagicJerk = 4000;

    // apply current limits
    leftShooterConfig.withCurrentLimits(limitsConfigs);
    rightShooterConfig.withCurrentLimits(limitsConfigs);
    kickerConfig.withCurrentLimits(limitsConfigs);

    // apply configs
    leftShooter.getConfigurator().apply(slot0Configs);
    leftShooter.getConfigurator().apply(leftShooterConfig);
    rightShooter.getConfigurator().apply(slot0Configs);
    rightShooter.getConfigurator().apply(rightShooterConfig);
    kicker.getConfigurator().apply(kickerConfig);

    // follow right shooter
    leftShooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightShooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightShooter.setControl(new Follower(ShooterConstants.leftShooterCANID, MotorAlignmentValue.Opposed));
    // kicker.setControl(new Follower(ShooterConstants.leftShooterCANID, MotorAlignmentValue.Aligned));

    //  music bs  
    // orchestra.addInstrument(leftShooter);
    // orchestra.addInstrument(rightShooter);
  }

  /* 
  public void playSong(String path){
    orchestra.loadMusic(path);
    orchestra.play();
  }
  public void stopSong(){
    orchestra.stop();
  }*/

  /**
   * shoot based on what launch calc spits out
   */
  public void autoShoot() {
    if (!aimer.inRange()) return; // dont shoot if not in range
    lights.addQueue(LightConstants.autoFirePriority);

    LaunchCalculator.getInstance().clearLaunchingParameters();
    targetRps = LaunchCalculator.getInstance().getParameters(swerve, -1.0).flywheelSpeed();
    this.setRps(targetRps);
  }

  public double getShooterAutoSpeed() {
    return targetRps;
  }

  @Override
  public void periodic() {
    if (leftShooter.get() == 0) {
      lights.removeQueue(LightConstants.autoFirePriority);
      lights.removeQueue(LightConstants.manualShooterPriority);
    }
    SmartDashboard.putNumber("Turret/Shooter/leftShooterSpeed", leftShooter.get());
    SmartDashboard.putNumber("Turret/Shooter/rightShooterSpeed", rightShooter.get());
    SmartDashboard.putNumber("Turret/Shooter/leftShooterRPS", leftShooter.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Turret/Shooter/rightShooterRPS", rightShooter.getVelocity().getValueAsDouble());
    
    SmartDashboard.putNumber("Turret/Shooter/kickerSpeed", kicker.get());
    SmartDashboard.putBoolean("Turret/Shooter/shooterRunning", shooterRunning);

    SmartDashboard.putNumber("Turret/Shooter/PID/setpoint", speedControl.getSetpoint());
    SmartDashboard.putNumber("Turret/Shooter/PID/error", speedControl.getError());

    SmartDashboard.putNumber("Turret/Shooter/LaunchCalc/Speed offset", LaunchCalculator.getSpeedOffset());
  }

  /**
   * set the shooter and kicker speed
   * @param speed the speed to set
   */
  public void setShooter(DoubleSupplier speed) {
    leftShooter.set(speed.getAsDouble());
  }

  /**
   * set the shooter and kicker speed
   * @param speed the speed to set
   */
  public void setShooter(Double speed) {
    leftShooter.set(speed);
  }

  /**
   * set the speed of the motor in rps
   * @param speed rps
   */
  public void setRps(double speed){
    leftShooter.setControl(velocityVoltage.withVelocity(speed));
    // rightShooter.setControl(velocityVoltage.withVelocity(speed)); // TODO: will follow?
    // leftShooter.setControl(motionMagicVelocityVoltage.withVelocity(speed));
    // rightShooter.setControl(motionMagicVelocityVoltage.withVelocity(speed));
    kicker.set(1);
  }

  /**
   * retrns the target rps
   * @return rps
   */
  public double getTargetRps() {
    return targetRps;
  }
  
  /**
   * check if the motor is at target rps
   * @return true if it is
   */
  public boolean atTargetRps() {
    return Math.abs(leftShooter.getVelocity().getValueAsDouble() - targetRps) < ShooterConstants.shooterErrorRps;
  }

  public void setRps(DoubleSupplier speed){
    setRps(speed.getAsDouble());
    lights.addQueue(LightConstants.manualShooterPriority);
  }

  public void stopShooter(){
    leftShooter.set(0.0);
    rightShooter.set(0.0);
    kicker.set(0.0);
  }
  /**
   * get the rps of the shooter
   */
  public Double getShooterRPS() {
    return leftShooter.getVelocity().getValueAsDouble();
  }

  public Double getShooterRealSpeed(){
    return leftShooter.get();
  }

  /*
    * Both the sysid commands are specific to one particular sysid routine, change
    * which one you're trying to characterize
    */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return sysIdRoutine.quasistatic(direction)
          .finallyDo(() -> stopShooter());
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return sysIdRoutine.dynamic(direction)
        .finallyDo(() -> stopShooter());
  }
}
