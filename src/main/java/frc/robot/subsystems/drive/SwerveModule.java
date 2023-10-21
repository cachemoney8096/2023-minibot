// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Calibrations;
import frc.robot.utils.AbsoluteEncoderChecker;
import frc.robot.utils.SparkMaxUtils;

public class SwerveModule implements Sendable {
  public final CANSparkMax drivingSparkMax;
  public final CANSparkMax turningSparkMax;

  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder turningEncoder;
  private AbsoluteEncoderChecker turningAbsoluteEncoderChecker = new AbsoluteEncoderChecker();

  private final SparkMaxPIDController drivingPIDController;
  private final SparkMaxPIDController turningPIDController;

  private double chassisAngularOffsetRadians = 0.0;
  private SwerveModuleState desiredState =
      new SwerveModuleState(
          ModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND, new Rotation2d());

  /**
   * Constructs a SwerveModule and configures the driving and turning motor, encoder, and PID
   * controller. This configuration is specific to the REV MAXSwerve Module built with NEOs, SPARKS
   * MAX, and a Through Bore Encoder.
   */
  public SwerveModule(int drivingCanId, int turningCanId, double chassisAngularOffset) {
    drivingSparkMax = new CANSparkMax(drivingCanId, MotorType.kBrushless);
    turningSparkMax = new CANSparkMax(turningCanId, MotorType.kBrushless);
    chassisAngularOffsetRadians = chassisAngularOffset;

    SparkMaxUtils.initWithRetry(this::initDriveSpark, Calibrations.SPARK_INIT_RETRY_ATTEMPTS);

    drivingEncoder = drivingSparkMax.getEncoder();
    drivingPIDController = drivingSparkMax.getPIDController();
    turningEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    turningPIDController = turningSparkMax.getPIDController();

    desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    drivingEncoder.setPosition(0);
  }

  public void initialize() {
    SparkMaxUtils.initWithRetry(this::initDriveSpark, Calibrations.SPARK_INIT_RETRY_ATTEMPTS);
    SparkMaxUtils.initWithRetry(this::initTurnSpark, Calibrations.SPARK_INIT_RETRY_ATTEMPTS);
  }

  /** Does all the initialization for the spark */
  boolean initTurnSpark() {
    int errors = 0;

    errors += SparkMaxUtils.check(turningSparkMax.restoreFactoryDefaults());

    turningSparkMax.setInverted(ModuleConstants.TURNING_SPARK_MAX_INVERTED);

    AbsoluteEncoder turningEncoderTmp = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    SparkMaxPIDController turningPidTmp = turningSparkMax.getPIDController();

    errors += SparkMaxUtils.check(turningPidTmp.setFeedbackDevice(turningEncoderTmp));

    errors += SparkMaxUtils.check(turningPidTmp.setP(ModuleCal.TURNING_P));
    errors += SparkMaxUtils.check(turningPidTmp.setI(ModuleCal.TURNING_I));
    errors += SparkMaxUtils.check(turningPidTmp.setD(ModuleCal.TURNING_D));
    errors += SparkMaxUtils.check(turningPidTmp.setFF(ModuleCal.TURNING_FF));

    /*  Gear ratio 1.0 because the encoder is 1:1 with the module (doesn't involve the actual turning
    / gear ratio)*/
    SparkMaxUtils.UnitConversions.setRadsFromGearRatio(
        turningEncoderTmp, ModuleConstants.TURNING_ENCODER_GEAR_RATIO);
    errors +=
        SparkMaxUtils.check(
            turningEncoderTmp.setInverted(ModuleConstants.TURNING_ENCODER_INVERTED));
    errors += SparkMaxUtils.check(turningPidTmp.setPositionPIDWrappingEnabled(true));
    errors +=
        SparkMaxUtils.check(
            turningPidTmp.setPositionPIDWrappingMinInput(
                ModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS));
    errors +=
        SparkMaxUtils.check(
            turningPidTmp.setPositionPIDWrappingMaxInput(
                ModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS));

    errors +=
        SparkMaxUtils.check(
            turningPidTmp.setOutputRange(
                ModuleCal.TURNING_MIN_OUTPUT, ModuleCal.TURNING_MAX_OUTPUT));
    errors +=
        SparkMaxUtils.check(turningSparkMax.setIdleMode(ModuleConstants.TURNING_MOTOR_IDLE_MODE));
    errors +=
        SparkMaxUtils.check(
            turningSparkMax.setSmartCurrentLimit(ModuleConstants.TURNING_MOTOR_CURRENT_LIMIT_AMPS));

    return errors == 0;
  }

  /** Does all the initialization for the spark */
  boolean initDriveSpark() {
    int errors = 0;
    errors += SparkMaxUtils.check(drivingSparkMax.restoreFactoryDefaults());

    drivingSparkMax.setInverted(ModuleConstants.DRIVING_SPARK_MAX_INVERTED);

    RelativeEncoder drivingEncoderTmp = drivingSparkMax.getEncoder();
    SparkMaxPIDController drivingPidTmp = drivingSparkMax.getPIDController();
    errors += SparkMaxUtils.check(drivingPidTmp.setFeedbackDevice(drivingEncoderTmp));

    errors += SparkMaxUtils.check(drivingPidTmp.setP(ModuleCal.DRIVING_P));
    errors += SparkMaxUtils.check(drivingPidTmp.setI(ModuleCal.DRIVING_I));
    errors += SparkMaxUtils.check(drivingPidTmp.setD(ModuleCal.DRIVING_D));
    errors += SparkMaxUtils.check(drivingPidTmp.setFF(ModuleCal.DRIVING_FF));

    errors +=
        SparkMaxUtils.check(
            drivingPidTmp.setOutputRange(
                ModuleCal.DRIVING_MIN_OUTPUT, ModuleCal.DRIVING_MAX_OUTPUT));

    errors +=
        SparkMaxUtils.check(
            drivingPidTmp.setOutputRange(
                ModuleCal.DRIVING_MIN_OUTPUT, ModuleCal.DRIVING_MAX_OUTPUT));

    errors +=
        SparkMaxUtils.check(
            drivingEncoderTmp.setPositionConversionFactor(
                ModuleConstants.DRIVING_ENCODER_POSITION_FACTOR_METERS));

    errors +=
        SparkMaxUtils.check(
            drivingEncoderTmp.setVelocityConversionFactor(
                ModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND));

    errors +=
        SparkMaxUtils.check(drivingSparkMax.setIdleMode(ModuleConstants.DRIVING_MOTOR_IDLE_MODE));
    errors +=
        SparkMaxUtils.check(
            drivingSparkMax.setSmartCurrentLimit(ModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT_AMPS));

    return errors == 0;
  }

  /**
   * Burns the current settings to sparks so they keep current settings on reboot. Should be done
   * after all settings are set.
   */
  public void burnFlashSparks() {
    Timer.delay(0.005);
    drivingSparkMax.burnFlash();
    Timer.delay(0.005);
    turningSparkMax.burnFlash();
  }

  /**
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
        drivingEncoder.getVelocity(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffsetRadians));
  }

  /**
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        drivingEncoder.getPosition(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffsetRadians));
  }

  /**
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle =
        desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffsetRadians));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState =
        SwerveModuleState.optimize(
            correctedDesiredState, new Rotation2d(turningEncoder.getPosition()));

    // Setting global desiredState to be optimized for the shuffleboard
    this.desiredState = optimizedDesiredState;

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivingPIDController.setReference(
        optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turningPIDController.setReference(
        optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetDrivingEncoders() {
    drivingEncoder.setPosition(0);
    ;
  }

  public double getTurningEncoderAbsPositionRad() {
    return turningEncoder.getPosition();
  }

  public void periodic() {
    turningAbsoluteEncoderChecker.addReading(turningEncoder.getPosition());
  }

  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Driving kP", drivingPIDController::getP, drivingPIDController::setP);
    builder.addDoubleProperty("Driving kI", drivingPIDController::getI, drivingPIDController::setI);
    builder.addDoubleProperty("Driving kD", drivingPIDController::getD, drivingPIDController::setD);
    builder.addDoubleProperty(
        "Driving kFF", drivingPIDController::getFF, drivingPIDController::setFF);
    builder.addDoubleProperty("Turning kP", turningPIDController::getP, turningPIDController::setP);
    builder.addDoubleProperty("Turning kI", turningPIDController::getI, turningPIDController::setI);
    builder.addDoubleProperty("Turning kD", turningPIDController::getD, turningPIDController::setD);
    builder.addDoubleProperty(
        "Turning kFF", turningPIDController::getFF, turningPIDController::setFF);
    builder.addDoubleProperty("Driving Vel (m/s)", drivingEncoder::getVelocity, null);
    builder.addDoubleProperty("Steering Pos (rad)", turningEncoder::getPosition, null);
    builder.addDoubleProperty(
        "Desired Vel (m/s)",
        () -> {
          return desiredState.speedMetersPerSecond;
        },
        null);
    builder.addDoubleProperty(
        "Desired Steer (rad)",
        () -> {
          return desiredState.angle.getRadians();
        },
        null);
    builder.addBooleanProperty(
        "Turning encoder connected", turningAbsoluteEncoderChecker::encoderConnected, null);
  }
}
