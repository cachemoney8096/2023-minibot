package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;

public class ArmCal {
  public static final double PLACEHOLDER_DOUBLE = 0.0;
  public static final float PLACEHOLDER_FLOAT = 0;
  public static final int PLACEHOLDER_INT = 0;

  /** Input deg, output Volts */
  public static final double ARM_P = PLACEHOLDER_DOUBLE,
      ARM_I = PLACEHOLDER_DOUBLE,
      ARM_D = PLACEHOLDER_DOUBLE;

  /** parameters for arm controller */
  public static final double
      ARM_MAX_VELOCITY_DEG_PER_SECOND =
          50.0, // 5880 rpm / (60 sec/min) * (360 deg/rev) / 135.4 = 250.56
      ARM_MAX_ACCELERATION_DEG_PER_SECOND_SQUARED =
          50.0; // angular accel = Torque / Inertia. 3.36 Nm * 135.4 / (7.26 * 0.3^2) kg-m^2 * (360
  // deg / 2pi
  // rad) = 39893 deg/s^2

  /** Arm positions in degrees for the armPositionMap */
  public static final double ARM_START_POSITION_DEG = 180.0,
      ARM_INTAKE_POSITION_DEG = 80.0,
      ARM_LOW_POSITION_DEG = 270.0,
      ARM_HIGH_MID_POSITION_DEG = 210.0,
      ARM_AVOID_LIMELIGHT_POSITION_DEG = 250.0;

  /** Absolute encoder position when the arm is at 0 degrees */
  public static double armAbsoluteEncoderZeroPosDeg = PLACEHOLDER_DOUBLE;

  /**
   * Margin for when we consider the arm has reached a position. This is logical (for considering
   * where the arm can go) but not functional (does not stop anything when reached). We apply
   * broader margins for the STARTING position as the arm transits through this position.
   */
  public static final double ARM_MARGIN_DEGREES = 2.0;

  /** Input deg/s, output volts. From recalc */
  public static final ArmFeedforward ARM_FEEDFORWARD = new ArmFeedforward(0.0, 0.53, 2.63, 0.02);

  public static final float ARM_NEGATIVE_LIMIT_DEGREES = 75;
  public static final float ARM_POSITIVE_LIMIT_DEGREES = 275;
  public static final int ARM_CURRENT_LIMIT_AMPS = 50;

  /** Various timeouts for auto lift movements */
  public static final double SCORE_TO_START_FAST_SEC = PLACEHOLDER_DOUBLE,
      SCORE_TO_START_SEC = PLACEHOLDER_DOUBLE,
      START_TO_PRESCORE_SEC = 5.0;
}
