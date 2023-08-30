// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.TreeMap;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.utils.AbsoluteEncoderChecker;
import frc.robot.utils.ScoringLocationUtil;
import frc.robot.utils.ScoringLocationUtil.ScoreHeight;

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
  private ArmPosition latestPosition = ArmPosition.STARTING;
  private ArmPosition goalPosition = ArmPosition.STARTING;
  private boolean cancelScore = false;
  private boolean scoringInProgress = false;

  private AbsoluteEncoderChecker armAbsoluteEncoderChecker = new AbsoluteEncoderChecker();

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
    armPositionMap.put(
        ArmPosition.AVOID_LIMELIGHT, ArmCal.ARM_AVOID_LIMELIGHT_POSITION_DEG);

  this.scoreLoc = scoreLoc;
  }

  public void initialize() {}

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    // Check if position has updated
    if (atPosition(ArmPosition.STARTING)) {
      this.latestPosition = ArmPosition.STARTING;
    } else if (atPosition(desiredPosition)) {
      this.latestPosition = desiredPosition;
    }

    controlPosition(desiredPosition);
    armAbsoluteEncoderChecker.addReading(armAbsoluteEncoder.getPosition());
  }

  /** Sets the desired position */
  public void setDesiredPosition(ArmPosition pos) {
    desiredPosition = pos;
  }

  public void startScore() {
    scoringInProgress = true;
    if (scoreLoc.getScoreHeight() == ScoreHeight.HIGH || scoreLoc.getScoreHeight() == ScoreHeight.MID) {
      setDesiredPosition(ArmPosition.SCORE_MID_HIGH);
    } else if (scoreLoc.getScoreHeight() == ScoreHeight.LOW) {
        setDesiredPosition(ArmPosition.SCORE_LOW);
    }
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

  /** True if the arm is at the current value of this.desiredPosition */
  private boolean atDesiredArmPosition() {
    double armMarginDegrees =
        desiredPosition == ArmPosition.STARTING
            ? ArmCal.ARM_START_MARGIN_DEGREES
            : ArmCal.ARM_MARGIN_DEGREES;
    double armPositionToCheckDegrees = armPositionMap.get(desiredPosition);
    double armPositionDegrees = armEncoder.getPosition();
    if (Math.abs(armPositionDegrees - armPositionToCheckDegrees) > armMarginDegrees) {
      return false;
    }
    return true;
  }

  /** True if the lift is at the queried position. */
  public boolean atPosition(ArmPosition positionToCheck) {
    double armMarginDegrees =
        positionToCheck == ArmPosition.STARTING
            ? ArmCal.ARM_START_MARGIN_DEGREES
            : ArmCal.ARM_MARGIN_DEGREES;
            
    double armPositionToCheckDegrees = armPositionMap.get(positionToCheck);
    double armPositionDegrees = armEncoder.getPosition();

    if (Math.abs(armPositionDegrees - armPositionToCheckDegrees) > armMarginDegrees) {
      return false;
    }
    return true;
  }

  /** Returns the cosine of the arm angle in degrees off of the horizontal. */
  public double getCosineArmAngle() {
    return Math.cos(
        Units.degreesToRadians(
            armEncoder.getPosition() - ArmConstants.ARM_POSITION_WHEN_HORIZONTAL_DEGREES));
  }

   /** Sends voltage commands to the arm and elevator motors, needs to be called every cycle */
   private void controlPosition(ArmPosition pos) {
    if (goalPosition != pos) {
      goalPosition = pos;
    }

    armController.setGoal(armPositionMap.get(pos));

    double armDemandVoltsA = armController.calculate(armEncoder.getPosition());
    double armDemandVoltsB =
        ArmCal.ARM_FEEDFORWARD.calculate(armController.getSetpoint().velocity);
    double armDemandVoltsC = ArmCal.ARBITRARY_ARM_FEED_FORWARD_VOLTS * getCosineArmAngle();
    armMotor.setVoltage(armDemandVoltsA + armDemandVoltsB + armDemandVoltsC);

    SmartDashboard.putNumber("Arm PID", armDemandVoltsA);
    SmartDashboard.putNumber("Arm FF", armDemandVoltsB);
    SmartDashboard.putNumber("Arm Gravity", armDemandVoltsC);
  }

  /**
   * if the robot has completed startScore() but hasn't started finishScore, then stop the robot from
   * scoring
   */
  public void cancelScore() {
    if (scoringInProgress) {
      setCancelScore(true);
    }
  }

  /** Runs instead of finishScore if cancelScore is true. */
  public void finishScoreCancelled() {
    setCancelScore(false);
    ManualPrepScoreSequence();
  }

  /** returns cancelScore (true if scoring action is cancelled) */
  public boolean getCancelScore() {
    return this.cancelScore;
  }

  /** sets cancelScore (true if scoring action is cancelled) */
  public void setCancelScore(boolean cancelled) {
    this.cancelScore = cancelled;
  }

  /** sets scoringInProgress */
  public void setScoringInProgress(boolean isScoring) {
    scoringInProgress = isScoring;
  }

  /**
   * takes the column and height from ScoringLocationUtil.java and converts that to a ArmPosition
   * then gives the position to the given arm
   */
  public void ManualPrepScoreSequence() {
    ScoreHeight height = scoreLoc.getScoreHeight();

    // low for all columns is the same height
    if (height == ScoreHeight.LOW) {
      setDesiredPosition(ArmPosition.SCORE_LOW);
    } else {
      setDesiredPosition(ArmPosition.SCORE_MID_HIGH);
    }
  }

  public void zeroArmAtCurrentPos() {
    ArmCal.ARM_ABSOLUTE_ENCODER_ZERO_POS_DEG = armAbsoluteEncoder.getPosition();
    System.out.println("New Zero for Arm: " + ArmCal.ARM_ABSOLUTE_ENCODER_ZERO_POS_DEG);
  }
}
