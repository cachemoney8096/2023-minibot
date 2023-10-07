// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.utils.ScoringLocationUtil;
import frc.robot.utils.ScoringLocationUtil.ScoreHeight;
import frc.robot.utils.SendableHelper;
import java.util.TreeMap;

public class Arm extends SubsystemBase {

  public enum ArmPosition {
    STARTING,
    INTAKE,
    SCORE_LOW,
    SCORE_MID_HIGH,
    AVOID_LIMELIGHT
  }

  public CANSparkMax armMotor =
      new CANSparkMax(RobotMap.ARM_PIVOT_MOTOR_CAN_ID, MotorType.kBrushless);

  private final RelativeEncoder armEncoder = armMotor.getEncoder();
  private final AbsoluteEncoder armAbsoluteEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);

  public ScoringLocationUtil scoreLoc;
  private ArmPosition desiredPosition = ArmPosition.STARTING;
  public boolean isScoring = false;

  TreeMap<ArmPosition, Double> armPositionMap;

  /** Input deg, output Volts */
  private ProfiledPIDController armController =
      new ProfiledPIDController(
          ArmCal.ARM_P,
          ArmCal.ARM_I,
          ArmCal.ARM_D,
          new TrapezoidProfile.Constraints(
              ArmCal.ARM_MAX_VELOCITY_DEG_PER_SECOND,
              ArmCal.ARM_MAX_ACCELERATION_DEG_PER_SECOND_SQUARED));

  public Arm(ScoringLocationUtil scoreLoc) {

    armPositionMap = new TreeMap<ArmPosition, Double>();
    armPositionMap.put(ArmPosition.STARTING, ArmCal.ARM_START_POSITION_DEG);
    armPositionMap.put(ArmPosition.INTAKE, ArmCal.ARM_INTAKE_POSITION_DEG);
    armPositionMap.put(ArmPosition.SCORE_LOW, ArmCal.ARM_LOW_POSITION_DEG);
    armPositionMap.put(ArmPosition.SCORE_MID_HIGH, ArmCal.ARM_HIGH_MID_POSITION_DEG);
    armPositionMap.put(ArmPosition.AVOID_LIMELIGHT, ArmCal.ARM_AVOID_LIMELIGHT_POSITION_DEG);

    this.scoreLoc = scoreLoc;
  }

  public void initialize() {}

  /** Sets the desired position */
  public void startScore() {
    isScoring = true;
    if (scoreLoc.getScoreHeight() == ScoreHeight.HIGH
        || scoreLoc.getScoreHeight() == ScoreHeight.MID) {
      goToPosition(ArmPosition.SCORE_MID_HIGH);
    } else if (scoreLoc.getScoreHeight() == ScoreHeight.LOW) {
      goToPosition(ArmPosition.SCORE_LOW);
    }
  }

  /** True if the arm is at the queried position. */
  public boolean atPosition(ArmPosition positionToCheck) {
    double armMarginDegrees = ArmCal.ARM_MARGIN_DEGREES;

    double armPositionToCheckDegrees = armPositionMap.get(positionToCheck);
    double armPositionDegrees = armEncoder.getPosition();

    return Math.abs(armPositionDegrees - armPositionToCheckDegrees) <= armMarginDegrees;
  }

  /** Returns the arm angle in degrees off of the horizontal. */
  public double getArmAngle() {
    return armEncoder.getPosition() - ArmConstants.ARM_POSITION_WHEN_HORIZONTAL_DEGREES;
  }

  /** Sends voltage commands to the arm and elevator motors */
  public void goToPosition(ArmPosition pos) {
    armController.setGoal(armPositionMap.get(pos));
    desiredPosition = pos;
    double armDemandVoltsA = armController.calculate(armEncoder.getPosition());
    double armDemandVoltsB = ArmCal.ARM_FEEDFORWARD.calculate(getArmAngle(), armController.getSetpoint().velocity);
    armMotor.setVoltage(armDemandVoltsA + armDemandVoltsB);
    SmartDashboard.putNumber("Arm PID", armDemandVoltsA);
    SmartDashboard.putNumber("Arm FF", armDemandVoltsB);
  }

  public void deployArmLessFar() {
    Double curAngle = armPositionMap.get(desiredPosition);
    Double newAngle = curAngle - 0.5;
    armPositionMap.replace(desiredPosition, newAngle);

    new PrintCommand("Latest angle for " + desiredPosition + ": " + newAngle);

    armController.setGoal(newAngle);
  }

  public void deployArmFurther() {
    Double curAngle = armPositionMap.get(desiredPosition);
    Double newAngle = curAngle + 0.5;
    armPositionMap.replace(desiredPosition, newAngle);

    new PrintCommand("Latest angle for " + desiredPosition + ": " + newAngle);

    armController.setGoal(newAngle);
  }
  /**
   * takes the column and height from ScoringLocationUtil.java and converts that to a ArmPosition
   * then gives the position to the given arm
   */
  public void ManualPrepScoreSequence() {
    ScoreHeight height = scoreLoc.getScoreHeight();

    // low for all columns is the same height
    if (height == ScoreHeight.LOW) {
      goToPosition(ArmPosition.SCORE_LOW);
    } else {
      goToPosition(ArmPosition.SCORE_MID_HIGH);
    }
  }

  public ScoreHeight getScoreHeight() {
    return scoreLoc.getScoreHeight();
  }

  public void zeroArmAtCurrentPos() {
    ArmCal.armAbsoluteEncoderZeroPosDeg = armAbsoluteEncoder.getPosition();
    System.out.println("New Zero for Arm: " + ArmCal.armAbsoluteEncoderZeroPosDeg);
  }

  public void setDegreesFromGearRatioAbsoluteEncoder(
      AbsoluteEncoder sparkMaxEncoder, double ratio) {
    double degreesPerRotation = 360.0 / ratio;
    double degreesPerRotationPerSecond = degreesPerRotation / 60.0;
    sparkMaxEncoder.setPositionConversionFactor(degreesPerRotation);
    sparkMaxEncoder.setVelocityConversionFactor(degreesPerRotationPerSecond);
  }

  public static void setDegreesFromGearRatioRelativeEncoder(
      RelativeEncoder sparkMaxEncoder, double ratio) {
    double degreesPerRotation = 360.0 / ratio;
    double degreesPerRotationPerSecond = degreesPerRotation / 60.0;
    sparkMaxEncoder.setPositionConversionFactor(degreesPerRotation);
    sparkMaxEncoder.setVelocityConversionFactor(degreesPerRotationPerSecond);
  }

  /** True if the arm is at the queried position. */
  public boolean atDesiredArmPosition() {
    double armMarginDegrees = ArmCal.ARM_MARGIN_DEGREES;
    double armPositionToCheckDegrees = armPositionMap.get(desiredPosition);
    double armPositionDegrees = armEncoder.getPosition();
    return Math.abs(armPositionDegrees - armPositionToCheckDegrees) <= armMarginDegrees;
  }

  @Override
  public void periodic(){
    
  }

  /**Cancellation function */
  public void cancelScore(){
      goToPosition(ArmPosition.STARTING);
      isScoring = false;
  }

  /** Output if we are scoring or not */
  public boolean currentlyScoring() {
    return isScoring;
  }

  /** Set whether we are scoring */
  public void setScoring(boolean scoring) {
    isScoring = scoring;
  }

  /** Does all the initialization for the sparks */
  public void initSparks() {

    armMotor.restoreFactoryDefaults();

    // inverting stuff
    armAbsoluteEncoder.setInverted(true);
    armMotor.setInverted(false);

    // Get positions and degrees of elevator through encoder in inches
    setDegreesFromGearRatioRelativeEncoder(armEncoder, ArmConstants.ARM_MOTOR_GEAR_RATIO);

    setDegreesFromGearRatioAbsoluteEncoder(armAbsoluteEncoder, 1.0);

    armMotor.setSoftLimit(SoftLimitDirection.kForward, ArmCal.ARM_POSITIVE_LIMIT_DEGREES);

    armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

    armMotor.setSoftLimit(SoftLimitDirection.kReverse, ArmCal.ARM_NEGATIVE_LIMIT_DEGREES);

    armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    armMotor.setIdleMode(IdleMode.kBrake);

    armMotor.setSmartCurrentLimit(ArmCal.ARM_CURRENT_LIMIT_AMPS);
  }

  /**
   * Burns the current settings to sparks so they keep current settings on reboot. Should be done
   * after all settings are set.
   */
  public void burnFlashSparks() {
    Timer.delay(0.005);
    armMotor.burnFlash();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    SendableHelper.addChild(builder, this, armController, "ArmController");

    builder.addDoubleProperty(
        "Arm Abs Position (deg)", armAbsoluteEncoder::getPosition, armEncoder::setPosition);
    builder.addBooleanProperty("Is scoring", this::currentlyScoring, this::setScoring);
    builder.addDoubleProperty(
        "Arm Position (deg)", armEncoder::getPosition, armEncoder::setPosition);
    builder.addDoubleProperty("Arm Vel (deg/s)", armEncoder::getVelocity, null);

    builder.addDoubleProperty("Arm output", armMotor::get, null);
    builder.addStringProperty(
        "Score Loc Height",
        () -> {
          return scoreLoc.getScoreHeight().toString();
        },
        null);

    builder.addBooleanProperty(
        "At desired position",
        () -> {
          return atPosition(desiredPosition);
        },
        null);
    builder.addBooleanProperty("At desired arm position", this::atDesiredArmPosition, null);

    builder.addStringProperty(
        "Desired position",
        () -> {
          return desiredPosition.toString();
        },
        null);
    builder.addStringProperty(
        "Score Loc Col",
        () -> {
          return scoreLoc.getScoreCol().toString();
        },
        null);
  }
}
