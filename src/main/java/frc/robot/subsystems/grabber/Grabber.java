package frc.robot.subsystems.grabber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.RobotMap;
import frc.robot.utils.SparkMaxUtils;
import frc.robot.utils.ScoringLocationUtil.ScoreHeight;
import frc.robot.utils.SparkMaxUtils;

public class Grabber extends SubsystemBase {

  private CANSparkMax frontMotor =
      new CANSparkMax(RobotMap.FRONT_INTAKE_ROLLER_MOTOR_CAN_ID, MotorType.kBrushless);

  public CANSparkMax backMotor =
      new CANSparkMax(RobotMap.BACK_INTAKE_ROLLER_MOTOR_CAN_ID, MotorType.kBrushless);

  private RelativeEncoder frontEncoder = frontMotor.getEncoder();
  private RelativeEncoder backEncoder = backMotor.getEncoder();

  private final DigitalInput gamePieceSensor =
      new DigitalInput(RobotMap.GRABBER_GAME_PIECE_SENSOR_DIO);

  private boolean runningCommand = false;

  private Command rumbleBriefly;

  private boolean sawObject = true;

  public Grabber(Command rumbleBrieflyCmd) {
    this.rumbleBriefly = rumbleBrieflyCmd;
    SparkMaxUtils.initWithRetry(this::initSparks, Calibrations.SPARK_INIT_RETRY_ATTEMPTS);
  }

  public void initialize() {
    SparkMaxUtils.initWithRetry(this::initSparks, Calibrations.SPARK_INIT_RETRY_ATTEMPTS);
  }

  public void setMotors(double power) {
    runningCommand = !(Math.abs(power) < 0.01);
    frontMotor.set(power);
    backMotor.set(power);
    System.out.println("spinning grabber motors at " + power);
    System.out.println("back motor: " + backMotor.get());
  }

  public void intake() {
    setMotors(GrabberCalibrations.INTAKING_POWER);
    System.out.println("intaking");
  }

  public void score(ScoreHeight height) {
    double scoringPower;
    if (height == ScoreHeight.LOW) {
      scoringPower = GrabberCalibrations.SCORE_LOW_POWER;
    } else if (height == ScoreHeight.MID) {
      scoringPower = GrabberCalibrations.SCORE_MID_POWER;
    } else {
      scoringPower = GrabberCalibrations.SCORE_HIGH_POWER;
    }
    setMotors(scoringPower);
  }

  public void eject() {
    setMotors(GrabberCalibrations.EJECTION_POWER);
  }

  public void stopMotors() {
    setMotors(0.0);
  }

  public boolean seeGamePiece() {
    // Sensor is false if there's a game piece
    boolean seeGamePieceNow = !gamePieceSensor.get();
    if (!sawObject && seeGamePieceNow) {
      rumbleBriefly.schedule();
    }

    sawObject = seeGamePieceNow;
    return seeGamePieceNow;
  }

  public boolean initSparks() {
    int errors = 0;
    errors += SparkMaxUtils.check(frontMotor.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(backMotor.restoreFactoryDefaults());

    errors +=
        SparkMaxUtils.check(
            frontMotor.setSmartCurrentLimit(GrabberCalibrations.MOTOR_CURRENT_LIMIT));
    errors +=
        SparkMaxUtils.check(
            backMotor.setSmartCurrentLimit(GrabberCalibrations.MOTOR_CURRENT_LIMIT));

    return errors == 0;
  }

  /**
   * Burns the current settings to sparks so they keep current settings on reboot. Should be done
   * after all settings are set.
   */
  public void burnFlashSparks() {
    Timer.delay(0.005);
    frontMotor.burnFlash();
    backMotor.burnFlash();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Front Motor Set Speed", frontMotor::get, frontMotor::set);
    builder.addDoubleProperty("Back Motor Set Speed", backMotor::get, backMotor::set);
    builder.addBooleanProperty("Sensor sees game piece", this::seeGamePiece, null);
    builder.addBooleanProperty("Running command", () -> {return this.runningCommand;}, null);
  }

  public void periodic() {
    if (seeGamePiece() && !runningCommand) {
      setMotors(GrabberCalibrations.HOLD_GAME_OBJECT_POWER);
    }
  }
}
