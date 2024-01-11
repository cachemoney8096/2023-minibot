package frc.robot;

public class RobotMap {
  //   /** Driving Motor CAN IDs */
  //   public static final int FRONT_LEFT_DRIVE_MOTOR_CAN_ID = 13,
  //       FRONT_RIGHT_DRIVE_MOTOR_CAN_ID = 19,
  //       BACK_LEFT_DRIVE_MOTOR_CAN_ID = 7,
  //       BACK_RIGHT_DRIVE_MOTOR_CAN_ID = 8;

  //   /** Steering Motor CAN IDs */
  //   public static final int FRONT_LEFT_STEERING_MOTOR_CAN_ID = 4,
  //       FRONT_RIGHT_STEERING_MOTOR_CAN_ID = 3,
  //       BACK_LEFT_STEERING_MOTOR_CAN_ID = 11,
  //       BACK_RIGHT_STEERING_MOTOR_CAN_ID = 15;

  /** Driving Motor CAN IDs */
  public static final int FRONT_LEFT_DRIVE_MOTOR_CAN_ID = 13,
      FRONT_RIGHT_DRIVE_MOTOR_CAN_ID = 3,
      BACK_LEFT_DRIVE_MOTOR_CAN_ID = 8,
      BACK_RIGHT_DRIVE_MOTOR_CAN_ID = 7;

  /** Steering Motor CAN IDs */
  public static final int FRONT_LEFT_STEERING_MOTOR_CAN_ID = 4,
      FRONT_RIGHT_STEERING_MOTOR_CAN_ID = 19,
      BACK_LEFT_STEERING_MOTOR_CAN_ID = 11,
      BACK_RIGHT_STEERING_MOTOR_CAN_ID = 15;

  /** Arm Pivot Motor CAN ID */
  public static final int ARM_PIVOT_MOTOR_CAN_ID = 5;

  /** Intake Roller Motor CAN IDs */
  public static final int FRONT_INTAKE_ROLLER_MOTOR_CAN_ID = 10,
      BACK_INTAKE_ROLLER_MOTOR_CAN_ID = 9;

  /** CANdle for the lights */
  public static final int CANDLE_CAN_ID = 50;

  /** Pigeon Gyro CAN ID */
  public static final int PIGEON_CAN_ID = 42;

  /** Channel for the game piece sensor */
  public static final int GRABBER_GAME_PIECE_SENSOR_DIO = 0;

  public static final int DRIVER_CONTROLLER_PORT = 0, OPERATOR_CONTROLLER_PORT = 1;
}
