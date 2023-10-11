package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.util.Units;

public class ModuleConstants {
  public static final boolean TURNING_ENCODER_INVERTED = true;

  public static final double WHEEL_DIAMETER_FUDGE_FACTOR = 1.0;

  public static final double NEO_MOTOR_FREE_SPEED_RPM = 5676.0;

  /** Calculations required for driving motor conversion factors and feed forward */
  public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NEO_MOTOR_FREE_SPEED_RPM / 60,
      WHEEL_DIAMETER_METERS = Units.inchesToMeters(3) * ModuleConstants.WHEEL_DIAMETER_FUDGE_FACTOR,
      WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

  public static final double DRIVING_MOTOR_REDUCTION = 4.71;
  public static final double DRIVE_WHEEL_FREE_SPEED_FUDGE_FACTOR = 1.0;
  public static final double DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND =
      DRIVE_WHEEL_FREE_SPEED_FUDGE_FACTOR
          * ((DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS) / DRIVING_MOTOR_REDUCTION);

  /** Meters */
  public static final double DRIVING_ENCODER_POSITION_FACTOR_METERS =
      WHEEL_CIRCUMFERENCE_METERS / DRIVING_MOTOR_REDUCTION;

  /** Meters per second */
  public static final double DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND =
      DRIVING_ENCODER_POSITION_FACTOR_METERS / 60.0;

  public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kCoast;
  public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

  public static final int DRIVING_MOTOR_CURRENT_LIMIT_AMPS = 50;

  public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS = 0.0; // radians
  public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS =
      2 * Math.PI; // radians

  public static final int TURNING_MOTOR_CURRENT_LIMIT_AMPS = 20;

  public static final boolean TURNING_SPARK_MAX_INVERTED = false;
  public static final boolean DRIVING_SPARK_MAX_INVERTED = true;

  public static final double TURNING_ENCODER_GEAR_RATIO = 1.0;
}
