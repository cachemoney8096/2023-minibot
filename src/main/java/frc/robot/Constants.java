// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.revrobotics.CANSparkMax.IdleMode;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int PLACEHOLDER_INT = 0;
  public static final double PLACEHOLDER_DOUBLE = 0.0;
  public static final float PLACEHOLDER_FLOAT = 0;


  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final int OPERATOR_CONTROLLER_PORT = 1;

  public static final int NUM_CANDLE_LEDS = PLACEHOLDER_INT;

  public static final class SwerveModule {
    public static final boolean TURNING_ENCODER_INVERTED = true;

    public static final double DRIVING_ENCODER_POSITION_FACTOR_METERS = PLACEHOLDER_DOUBLE; // meters
    public static final double DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND = PLACEHOLDER_DOUBLE; // meters per second

    public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kCoast;
    public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake; 
    
    public static final int DRIVING_MOTOR_CURRENT_LIMIT_AMPS = PLACEHOLDER_INT;

    public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS = 0; // radians
    public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS =
        2 * Math.PI; // radians

    public static final int TURNING_MOTOR_CURRENT_LIMIT_AMPS = PLACEHOLDER_INT;


  }
}
