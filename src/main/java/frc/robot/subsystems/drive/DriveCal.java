package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;

public class DriveCal {
  /** For the purposes of trajectory constraints */
  public static final double MAX_LINEAR_SPEED_METERS_PER_SEC = 4.4,
      MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ = 3.0;

  /** For the purposes of trajectory constraints */
  public static final double MEDIUM_LINEAR_SPEED_METERS_PER_SEC = 2.5,
      MEDIUM_LINEAR_ACCELERATION_METERS_PER_SEC_SQ = 2.5;

  /** For the purposes of trajectory constraints */
  public static final double SLOW_LINEAR_SPEED_METERS_PER_SEC = 2.0,
      SLOW_LINEAR_ACCELERATION_METERS_PER_SEC_SQ = 2.0;

  /** For the purposes of trajectory constraints */
  public static final double VERY_SLOW_LINEAR_SPEED_METERS_PER_SEC = 1.5,
      VERY_SLOW_LINEAR_ACCELERATION_METERS_PER_SEC_SQ = 2.0;

  /**
   * Angular offset of the modules relative to the zeroing fixture in radians. Ideally should be
   * relative to the ficture but they are actually slightly different.
   */
  public static double SWERVE_FRONT_LEFT_ANGULAR_OFFSET_RAD = 3.575,
      SWERVE_FRONT_RIGHT_ANGULAR_OFFSET_RAD = 1.177,
      SWERVE_BACK_LEFT_ANGULAR_OFFSET_RAD = 5.235,
      SWERVE_BACK_RIGHT_ANGULAR_OFFSET_RAD = 4.677;

  /**
   * Angular offsets of the modules relative to the chassis in radians. The modules form an O when
   * fixtured, so they are iteratively 90 deg from each other.
   */
  public static final double
      FRONT_LEFT_CHASSIS_ANGULAR_OFFSET_RAD =
          SWERVE_FRONT_LEFT_ANGULAR_OFFSET_RAD,
      FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD =
          SWERVE_FRONT_RIGHT_ANGULAR_OFFSET_RAD,
      BACK_LEFT_CHASSIS_ANGULAR_OFFSET_RAD =
          SWERVE_BACK_LEFT_ANGULAR_OFFSET_RAD,
      BACK_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD =
          SWERVE_BACK_RIGHT_ANGULAR_OFFSET_RAD;

  /** Controller on module speed for rotating to target, input degrees [-180,180], output [0,1]. */
  public static final PIDController ROTATE_TO_TARGET_PID_CONTROLLER =
      new PIDController(0.015, 0, 0.000); // From 2023 (from 2022)

  /** Feed forward for rotating to target, gets added to or subtracted from PID controller. */
  public static final double ROTATE_TO_TARGET_FF = 0.01; // from 2023

  /** Auton path finding controllers */
  public static final PIDController PATH_X_CONTROLLER = new PIDController(9.0, 0.0, 0.0),
      PATH_Y_CONTROLLER = new PIDController(9.0, 0.0, 0.0);

  /** High profile constraints = pure P controller */
  public static final PIDController PATH_THETA_CONTROLLER = new PIDController(9.0, 0.0, 0.80);

  /** If the desired chassis rotation is below this value in [0,1], it is ignored */
  public static final double ROTATION_DEADBAND_THRESHOLD = 0.04;

  /** Drive speed multipliers for teleop intaking and scoring */
  public static final double THROTTLE_FOR_INTAKING = 1.0,
      THROTTLE_FOR_SCORING_AND_SHELF = 0.35,
      THROTTLE_FOR_SLOW_BUTTON = 0.5;

  public static final double IMU_PITCH_BIAS_DEG = 1.0;

  public static final double LEFT_ANGLE_CARDINAL_DIRECTION = 77,
      RIGHT_ANGLE_CARDINAL_DIRECTION = 77;
}
