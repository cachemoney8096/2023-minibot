package frc.robot.subsystems.grabber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Grabber extends SubsystemBase {

  private CANSparkMax frontMotor =
      new CANSparkMax(RobotMap.FRONT_INTAKE_ROLLER_MOTOR_CAN_ID, MotorType.kBrushless);

  private CANSparkMax backMotor =
      new CANSparkMax(RobotMap.BACK_INTAKE_ROLLER_MOTOR_CAN_ID, MotorType.kBrushless);

  private final DigitalInput gamePieceSensor = new DigitalInput(RobotMap.LIFT_GAME_PIECE_DIO);
  
  public void spinMotors(double power) {
    frontMotor.set(power);
    backMotor.set(power);
  }

  public void intake() {
    spinMotors(GrabberCalibrations.INTAKING_POWER);
  }

  public void scoreHigh() {
    spinMotors(GrabberCalibrations.SCORE_HIGH_POWER);
  }

  public void scoreMid() {
    spinMotors(GrabberCalibrations.SCORE_MID_POWER);
  }

  public void scoreLow() {
    spinMotors(GrabberCalibrations.SCORE_LOW_POWER);
  }

  public void eject() {
    spinMotors(GrabberCalibrations.EJECTION_POWER);
  }

  public void stopIntakingGamePiece() {
    spinMotors(0.0);
  }

  public boolean seeGamePiece(){
    boolean seeGamePieceNow = !gamePieceSensor.get();
    return seeGamePieceNow;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Front Motor Set Speed", frontMotor::get, frontMotor::set);
    builder.addDoubleProperty("Back Motor Set Speed", backMotor::get, backMotor::set);
    builder.addBooleanProperty("Sensor sees game piece", this::seeGamePiece, null);
  }
  public void periodic() {}
}
