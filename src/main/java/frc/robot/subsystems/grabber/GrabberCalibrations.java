package frc.robot.subsystems.grabber;

public final class GrabberCalibrations {
  public static final double PLACEHOLDER_DOUBLE = 0.0;

  /* motor spin powers for various actions */
  public static final double INTAKING_POWER = 1.0,
      EJECTION_POWER = -1.0,
      SCORE_HIGH_POWER = PLACEHOLDER_DOUBLE,
      SCORE_MID_POWER = PLACEHOLDER_DOUBLE,
      SCORE_LOW_POWER = PLACEHOLDER_DOUBLE;

  /* wait times spinning motors for various actions*/
  public static final double SCORING_WAIT_TIME = PLACEHOLDER_DOUBLE,
      EJECTION_WAIT_TIME = PLACEHOLDER_DOUBLE;

  /* motor max current*/
  public static final int MOTOR_CURRENT_LIMIT = 30;

  /*Motor power to hold game object */
  public static final double HOLD_GAME_OBJECT_POWER = 0.1;
}
