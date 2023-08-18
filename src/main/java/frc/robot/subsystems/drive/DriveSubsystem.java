// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private CANSparkMax FRONT_LEFT_DRIVE_MOTOR = new CANSparkMax(RobotMap.FRONT_LEFT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless);
  private CANSparkMax FRONT_RIGHT_DRIVE_MOTOR = new CANSparkMax(RobotMap.FRONT_RIGHT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless);
  
  private CANSparkMax BACK_LEFT_DRIVE_MOTOR = new CANSparkMax(RobotMap.BACK_LEFT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless);
  private CANSparkMax BACK_RIGHT_DRIVE_MOTOR = new CANSparkMax(RobotMap.BACK_RIGHT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless);

  public DriveSubsystem() {}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
