// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.util.Units;

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

    public static final double WHEEL_DIAMETER_FUDGE_FACTOR = 0.898 * 1.120 * 0.992 * 1.035;
    
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3) * WHEEL_DIAMETER_FUDGE_FACTOR,
        WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    public static final double DRIVING_MOTOR_REDUCTION = 4.8;
    /** Meters */
    public static final double DRIVING_ENCODER_POSITION_FACTOR_METERS =
        WHEEL_CIRCUMFERENCE_METERS / DRIVING_MOTOR_REDUCTION; 
    /** Meters per second */
    public static final double DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND =
        DRIVING_ENCODER_POSITION_FACTOR_METERS / 60.0; 

    public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kCoast;
    public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake; 
    
    public static final int DRIVING_MOTOR_CURRENT_LIMIT_AMPS = 50;

    public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS = 0; // radians
    public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS =
        2 * Math.PI; // radians

    public static final int TURNING_MOTOR_CURRENT_LIMIT_AMPS = 20;

    public static final boolean TURNING_SPARK_MAX_INVERTED = false;
    public static final boolean DRIVING_SPARK_MAX_INVERTED = true;

    public static final double TURNING_ENCODER_GEAR_RATIO = 46.2;
  }
}
