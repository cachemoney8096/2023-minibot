package frc.robot.subsystems.drive;

public class ModuleCal {
  public static final double PLACEHOLDER_DOUBLE = 0.0;

  /** Values from 2023 */
  public static final double DRIVING_P = 0.1,
      DRIVING_I = 0.0,
      DRIVING_D = 0.1,
      DRIVING_FF = 0.95 / ModuleConstants.DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND;

  /** Values from 2023 */
  public static final double TURNING_P = 0.8, TURNING_I = 0.0, TURNING_D = 0.1, TURNING_FF = 0.0;

  public static final double DRIVING_MIN_OUTPUT = -1.0, DRIVING_MAX_OUTPUT = 1.0;
  public static final double TURNING_MIN_OUTPUT = -1.0, TURNING_MAX_OUTPUT = 1.0;
}
