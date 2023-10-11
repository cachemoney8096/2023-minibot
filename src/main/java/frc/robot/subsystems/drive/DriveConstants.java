package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  /**
   * Driving Parameters - Note that these are not the maximum capable speeds of the robot, rather
   * the allowed maximum speeds
   */
  public static final double MAX_SPEED_METERS_PER_SECOND = 4.3,
      MAX_ANGULAR_SPEED_RAD_PER_SECONDS = 2 * Math.PI; // radians per second

  /** Chassis configuration */

  /** Distance between centers of right and left wheels on robot */
  public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(7.5);

  /** Distance between front and back wheels on robot */
  public static final double WHEEL_BASE_METERS = Units.inchesToMeters(14.5);

  public static final SwerveDriveKinematics DRIVE_KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
          new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
          new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
          new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2));

  public static final boolean GYRO_REVERSED = false;
}
