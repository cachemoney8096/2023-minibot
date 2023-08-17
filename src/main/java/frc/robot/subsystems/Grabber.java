package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {
    private CANSparkMax FRONT_LEFT_DRIVE_MOTOR = new CANSparkMax(RobotMap.FRONT_LEFT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless);
    private CANSparkMax FRONT_RIGHT_DRIVE_MOTOR = new CANSparkMax(RobotMap.FRONT_RIGHT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless);
    private CANSparkMax BACK_LEFT_DRIVE_MOTOR = new CANSparkMax(RobotMap.BACK_LEFT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless);
    private CANSparkMax BACK_RIGHT_DRIVE_MOTOR = new CANSparkMax(RobotMap.BACK_RIGHT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless);

    public void intake() {};
    public void scoreHigh() {};
    public void scoreMid() {};
    public void scoreLow() {};

    @Override
    public void periodic() {};
}
