// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;
import com.fasterxml.jackson.databind.ser.impl.ReadOnlyClassToSerializerMap;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Lights;
import frc.robot.utils.GeometryUtils;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class DriveSubsystem extends SubsystemBase {
  private double targetHeadingDegrees;

  private Lights lights;

  // Create SwerveModules
  public final SwerveModule frontLeft =
      new SwerveModule(
          RobotMap.FRONT_LEFT_DRIVE_MOTOR_CAN_ID,
          RobotMap.FRONT_LEFT_STEERING_MOTOR_CAN_ID,
          DriveCal.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET_RAD);

  public final SwerveModule frontRight =
      new SwerveModule(
          RobotMap.FRONT_RIGHT_DRIVE_MOTOR_CAN_ID,
          RobotMap.FRONT_RIGHT_STEERING_MOTOR_CAN_ID,
          DriveCal.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD);

  public final SwerveModule backLeft =
      new SwerveModule(
          RobotMap.BACK_LEFT_DRIVE_MOTOR_CAN_ID,
          RobotMap.BACK_LEFT_STEERING_MOTOR_CAN_ID,
          DriveCal.BACK_LEFT_CHASSIS_ANGULAR_OFFSET_RAD);

  public final SwerveModule backRight =
      new SwerveModule(
          RobotMap.BACK_RIGHT_DRIVE_MOTOR_CAN_ID,
          RobotMap.BACK_RIGHT_STEERING_MOTOR_CAN_ID,
          DriveCal.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD);

  // The gyro sensor
  private final WPI_Pigeon2 gyro = new WPI_Pigeon2(RobotMap.PIGEON_CAN_ID);
  private ChassisSpeeds lastSetChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  public Optional<Pose2d> targetPose = Optional.empty();
  public boolean generatedPath = false;
  private MedianFilter pitchFilter = new MedianFilter(5);
  private double latestFilteredPitchDeg = 0.0;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          DriveConstants.DRIVE_KINEMATICS, Rotation2d.fromDegrees(0.0), getModulePositions());

  /** Multiplier for drive speed, does not affect trajectory following */
  private double throttleMultiplier = 1.0;

  private BooleanSupplier isTimedMatch;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(Lights lightsSubsystem, BooleanSupplier isTimedMatchFunc) {
    gyro.configFactoryDefault();
    gyro.reset();
    gyro.configMountPose(AxisDirection.PositiveY, AxisDirection.PositiveZ);
    lights = lightsSubsystem;
    isTimedMatch = isTimedMatchFunc;
  }

  public void initialize() {
    frontLeft.initialize();
    frontRight.initialize();
    backLeft.initialize();
    backRight.initialize();
  }

  public double getFilteredPitch() {
    return latestFilteredPitchDeg - DriveCal.IMU_PITCH_BIAS_DEG;
  }

  @Override
  public void periodic() {
    latestFilteredPitchDeg = pitchFilter.calculate(gyro.getPitch());

    // Update the odometry in the periodic block
    frontLeft.periodic();
    frontRight.periodic();
    backLeft.periodic();
    backRight.periodic();
    odometry.update(Rotation2d.fromDegrees(gyro.getYaw()), getModulePositions());
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    };
  }

  public void initSparks() {
    frontLeft.initDriveSpark();
    frontRight.initDriveSpark();
    backLeft.initDriveSpark();
    backRight.initDriveSpark();
    frontLeft.initTurnSpark();
    frontRight.initTurnSpark();
    backLeft.initTurnSpark();
    backRight.initTurnSpark();
  }

  public void burnFlashSparks() {
    frontLeft.burnFlashSparks();
    frontRight.burnFlashSparks();
    backLeft.burnFlashSparks();
    backRight.burnFlashSparks();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    // Just update the translation, not the yaw
    Pose2d resetPose = new Pose2d(pose.getTranslation(), Rotation2d.fromDegrees(gyro.getYaw()));
    odometry.resetPosition(Rotation2d.fromDegrees(gyro.getYaw()), getModulePositions(), resetPose);
  }

  public void resetYawToAngle(double yawDeg) {
    double curYawDeg = gyro.getYaw();
    double offsetToTargetDeg = targetHeadingDegrees - curYawDeg;
    gyro.setYaw(yawDeg);
    Pose2d curPose = getPose();
    Pose2d resetPose = new Pose2d(curPose.getTranslation(), Rotation2d.fromDegrees(yawDeg));
    odometry.resetPosition(Rotation2d.fromDegrees(yawDeg), getModulePositions(), resetPose);
    targetHeadingDegrees = yawDeg + offsetToTargetDeg;
  }

  public void resetYaw() {
    resetYawToAngle(0.0);
  }

  /**
   * Correction for swerve second order dynamics issue. Borrowed from 254:
   * https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Drive.java#L325
   * Discussion:
   * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
   */
  private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
    final double LOOP_TIME_S = 0.02;
    Pose2d futureRobotPose =
        new Pose2d(
            originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
            originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
            Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
    Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
    ChassisSpeeds updatedSpeeds =
        new ChassisSpeeds(
            twistForPose.dx / LOOP_TIME_S,
            twistForPose.dy / LOOP_TIME_S,
            twistForPose.dtheta / LOOP_TIME_S);
    return updatedSpeeds;
  }

  /** Keep modules in current position, don't drive */
  public void setNoMove() {
    Rotation2d frontLeftCurrRot = frontLeft.getPosition().angle;
    Rotation2d frontRightCurrRot = frontRight.getPosition().angle;
    Rotation2d backLeftCurrRot = backLeft.getPosition().angle;
    Rotation2d backRightCurrRot = backRight.getPosition().angle;
    frontLeft.setDesiredState(new SwerveModuleState(0, frontLeftCurrRot));
    frontRight.setDesiredState(new SwerveModuleState(0, frontRightCurrRot));
    backLeft.setDesiredState(new SwerveModuleState(0, backLeftCurrRot));
    backRight.setDesiredState(new SwerveModuleState(0, backRightCurrRot));
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Desired speed of the robot in the x direction (forward), [-1,1].
   * @param ySpeed Desired speed of the robot in the y direction (sideways), [-1,1].
   * @param rot Desired angular rate of the robot, [-1,1].
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (isTimedMatch.getAsBoolean()
        && DriverStation.isTeleop()
        && DriverStation.getMatchTime() < 0.3) {
      setX();
      return;
    }

    // x, y, and rot are all being deadbanded from 0.1 to 0.0, so checking if
    // they're equal to 0
    // does account for controller deadzones.
    if (xSpeed == 0 && ySpeed == 0 && rot == 0) {
      setNoMove();
      return;
    }

    // Adjust input based on max speed
    xSpeed *= DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    ySpeed *= DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    rot *= DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_SECONDS;

    xSpeed *= throttleMultiplier;
    ySpeed *= throttleMultiplier;
    rot *= throttleMultiplier;

    ChassisSpeeds desiredChassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rot, Rotation2d.fromDegrees(gyro.getYaw()))
            : new ChassisSpeeds(xSpeed, ySpeed, rot);

    desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);
    lastSetChassisSpeeds = desiredChassisSpeeds;

    var swerveModuleStates =
        DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(desiredChassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    lights.setPartyMode();
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    targetHeadingDegrees = getHeadingDegrees();
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetDrivingEncoders() {
    frontLeft.resetDrivingEncoders();
    backLeft.resetDrivingEncoders();
    frontRight.resetDrivingEncoders();
    backRight.resetDrivingEncoders();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeadingDegrees() {
    return Rotation2d.fromDegrees(gyro.getYaw()).getDegrees();
  }

  /**
   * Keeps the heading of the robot when the driver is not turning, by using PID to keep the
   * distance between the actual heading and the last intended heading to 0.
   *
   * @param x Desired speed of the robot in the x direction (forward), [-1,1].
   * @param y Desired speed of the robot in the y direction (sideways), [-1,1].
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void keepHeading(double x, double y, boolean fieldRelative) {
    double currentHeadingDegrees = getHeadingDegrees();
    double headingDifferenceDegrees = currentHeadingDegrees - targetHeadingDegrees;
    double offsetHeadingDegrees = MathUtil.inputModulus(headingDifferenceDegrees, -180, 180);

    double pidRotation =
        DriveCal.ROTATE_TO_TARGET_PID_CONTROLLER.calculate(offsetHeadingDegrees, 0.0);
    double ffRotation = Math.signum(offsetHeadingDegrees) * DriveCal.ROTATE_TO_TARGET_FF;

    double desiredRotation = pidRotation - ffRotation;

    if (Math.abs(desiredRotation) < DriveCal.ROTATION_DEADBAND_THRESHOLD) {
      desiredRotation = 0;
    }

    drive(x, y, desiredRotation, fieldRelative);
  }

  public int convertCardinalDirections(int povAngleDeg) {
    // change d-pad values for left and right to specified angle
    // povAngleDeg starts at 0 up and increases clockwise, so 270 is left and 90 is right
    if (povAngleDeg == 270) {
      povAngleDeg += DriveCal.LEFT_ANGLE_CARDINAL_DIRECTION;
    } else if (povAngleDeg == 90) {
      povAngleDeg -= DriveCal.RIGHT_ANGLE_CARDINAL_DIRECTION;
    }
    // targetHeadingDegrees is counterclockwise so need to flip povAngle
    povAngleDeg = 360 - povAngleDeg;
    return povAngleDeg;
  }

  /**
   * Determines whether to rotate according to input or to run the keep heading code, by checking if
   * the (already deadbanded) rotation input is equal to 0.
   *
   * @param x Desired speed of the robot in the x direction (forward), [-1,1].
   * @param y Desired speed of the robot in the y direction (sideways), [-1,1].
   * @param rot Desired angular rate of the robot, [-1,1].
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param povAngleDeg Get the angle in degrees of the D-pad (clockwise, -1 means POV not pressed).
   */
  public void rotateOrKeepHeading(
      double x, double y, double rot, boolean fieldRelative, int povAngleDeg) {
    if (povAngleDeg != -1) {
      targetHeadingDegrees = convertCardinalDirections(povAngleDeg);
      keepHeading(x, y, fieldRelative);
    } else if (rot == 0) {
      keepHeading(x, y, fieldRelative);
    } else {
      targetHeadingDegrees = getHeadingDegrees();
      drive(x, y, rot, fieldRelative);
    }
  }

  public void setForward() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  }

  public WPI_Pigeon2 getGyro() {
    return gyro;
  }

  /** Taken from Github */
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {

    return new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  // Reset odometry for the first path you run during auto
                  if (isFirstPath) {
                    this.resetOdometry(traj.getInitialHolonomicPose());
                  }
                }),
            new PPSwerveControllerCommand(
                traj,
                this::getPose, // Pose supplier
                DriveConstants.DRIVE_KINEMATICS,
                DriveCal.PATH_X_CONTROLLER,
                DriveCal.PATH_Y_CONTROLLER, // usually the same values as X controller
                DriveCal.PATH_THETA_CONTROLLER,
                this::setModuleStates, // Module states consumer
                false, // Should the path be automatically mirrored depending on alliance color.
                this // Requires this drive subsystem
                ),
            new InstantCommand(
                () -> {
                  targetHeadingDegrees = getHeadingDegrees();
                }),
            new PrintCommand("Finished a trajectory"))
        .withName("Follow trajectory");
  }

  public Pose2d getPastPose(double latencySec) {
    Pose2d curPose = getPose();
    double latencyAdjustmentSec = 0.00;
    latencySec += latencyAdjustmentSec;
    Transform2d pastTransform =
        new Transform2d(
            new Translation2d(
                -lastSetChassisSpeeds.vxMetersPerSecond * latencySec,
                -lastSetChassisSpeeds.vyMetersPerSecond * latencySec),
            Rotation2d.fromRadians(lastSetChassisSpeeds.omegaRadiansPerSecond * latencySec)
                .unaryMinus());
    Pose2d pastPose = curPose.plus(pastTransform);
    return pastPose;
  }

  public void setLimelightTargetFromTransform(Transform2d transform, double latencySec) {
    // Transform is to get the limelight to the correct location, not to get the robot
    // Here we correct for that
    Transform2d flipTransform =
        new Transform2d(
            new Translation2d(-transform.getX(), transform.getY()), transform.getRotation());
    System.out.println("Flip Transform: " + flipTransform.getX() + " " + flipTransform.getY());

    Pose2d curPose = getPose();
    double latencyAdjustmentSec = 0.00;
    latencySec += latencyAdjustmentSec;
    Transform2d pastTransform =
        new Transform2d(
            new Translation2d(
                -lastSetChassisSpeeds.vxMetersPerSecond * latencySec,
                -lastSetChassisSpeeds.vyMetersPerSecond * latencySec),
            Rotation2d.fromRadians(lastSetChassisSpeeds.omegaRadiansPerSecond * latencySec)
                .unaryMinus());
    Pose2d pastPose = curPose.plus(pastTransform);

    final boolean useLatencyAdjustment = true;

    targetPose =
        useLatencyAdjustment
            ? Optional.of(pastPose.plus(flipTransform))
            : Optional.of(curPose.plus(flipTransform));
  }

  public PathPlannerTrajectory pathToPoint(Pose2d finalPose, double finalSpeedMetersPerSec) {
    Pose2d curPose = getPose();
    Transform2d finalTransform =
        new Transform2d(finalPose.getTranslation(), finalPose.getRotation());
    System.out.println(
        "Trajectory Transform: " + finalTransform.getX() + " " + finalTransform.getY());
    Rotation2d finalHeading = Rotation2d.fromDegrees(180);
    Rotation2d finalHolonomicRotation = finalPose.getRotation();
    PathPlannerTrajectory path =
        PathPlanner.generatePath(
            new PathConstraints(
                DriveCal.MEDIUM_LINEAR_SPEED_METERS_PER_SEC,
                DriveCal.MEDIUM_LINEAR_ACCELERATION_METERS_PER_SEC_SQ),
            PathPoint.fromCurrentHolonomicState(curPose, lastSetChassisSpeeds),
            new PathPoint(
                    finalTransform.getTranslation(),
                    finalHeading,
                    finalHolonomicRotation,
                    finalSpeedMetersPerSec)
                .withPrevControlLength(0.01));
    return path;
  }

  public Optional<PathPlannerTrajectory> poseToPath() {
    Pose2d curPose = getPose();
    double coastLatencySec = 0.00;
    Transform2d coastTransform =
        new Transform2d(
            new Translation2d(
                lastSetChassisSpeeds.vxMetersPerSecond * coastLatencySec,
                lastSetChassisSpeeds.vyMetersPerSecond * coastLatencySec),
            Rotation2d.fromRadians(lastSetChassisSpeeds.omegaRadiansPerSecond * coastLatencySec));
    Pose2d futurePose = curPose.plus(coastTransform);

    System.out.println("Acquired target? " + targetPose.isPresent());
    if (!targetPose.isPresent()) {
      return Optional.empty();
    }

    Pose2d finalPose = targetPose.get();
    Transform2d finalTransform =
        new Transform2d(finalPose.getTranslation(), finalPose.getRotation());
    Rotation2d finalHeading = Rotation2d.fromDegrees(180);
    Rotation2d finalHolonomicRotation = Rotation2d.fromDegrees(0);
    PathPlannerTrajectory path =
        PathPlanner.generatePath(
            new PathConstraints(
                DriveCal.SLOW_LINEAR_SPEED_METERS_PER_SEC,
                DriveCal.SLOW_LINEAR_ACCELERATION_METERS_PER_SEC_SQ),
            PathPoint.fromCurrentHolonomicState(futurePose, lastSetChassisSpeeds),
            new PathPoint(finalTransform.getTranslation(), finalHeading, finalHolonomicRotation)
                .withPrevControlLength(0.5));
    generatedPath = true;
    return Optional.of(path);
  }

  /** Driving inputs will get multiplied by the throttle value, so it should be in [0,1] */
  public void throttle(double throttleValue) {
    throttleMultiplier = throttleValue;
  }

  public void offsetCurrentHeading(double offsetDegrees) {
    targetHeadingDegrees = getHeadingDegrees() + offsetDegrees;
  }

  public void setZeroTargetHeading() {
    targetHeadingDegrees = 0.0;
  }

  public void stopDriving() {
    drive(0, 0, 0, true);
  }

  public Command stopDrivingCommand() {
    return new InstantCommand(this::stopDriving, this);
  }

  public Command turnInPlace(double timeoutSec) {
    return new RunCommand(
            () -> {
              rotateOrKeepHeading(0, 0, 0, true, -1);
            })
        .withTimeout(timeoutSec);
  }

  public void zeroFrontLeftAtCurrentPos() {
    DriveCal.SWERVE_FRONT_LEFT_ANGULAR_OFFSET_RAD = frontLeft.getTurningEncoderAbsPositionRad();
    System.out.println(
        "New Zero for Front Left Swerve: " + DriveCal.SWERVE_FRONT_LEFT_ANGULAR_OFFSET_RAD);
  }

  public void zeroFrontRightAtCurrentPos() {
    DriveCal.SWERVE_FRONT_RIGHT_ANGULAR_OFFSET_RAD = frontRight.getTurningEncoderAbsPositionRad();
    System.out.println(
        "New Zero for Front Right Swerve: " + DriveCal.SWERVE_FRONT_RIGHT_ANGULAR_OFFSET_RAD);
  }

  public void zeroBackLeftAtCurrentPos() {
    DriveCal.SWERVE_BACK_LEFT_ANGULAR_OFFSET_RAD = backLeft.getTurningEncoderAbsPositionRad();
    System.out.println(
        "New Zero for Back Left Swerve: " + DriveCal.SWERVE_BACK_LEFT_ANGULAR_OFFSET_RAD);
  }

  public void zeroBackRightAtCurrentPos() {
    DriveCal.SWERVE_BACK_RIGHT_ANGULAR_OFFSET_RAD = backRight.getTurningEncoderAbsPositionRad();
    System.out.println(
        "New Zero for Back Right Swerve: " + DriveCal.SWERVE_BACK_RIGHT_ANGULAR_OFFSET_RAD);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    addChild("X Controller", DriveCal.PATH_X_CONTROLLER);
    addChild("Y Controller", DriveCal.PATH_Y_CONTROLLER);
    addChild("Theta Controller", DriveCal.PATH_THETA_CONTROLLER);
    addChild("Rotate to target controller", DriveCal.ROTATE_TO_TARGET_PID_CONTROLLER);
    addChild("Front Right", frontRight);
    addChild("Front Left", frontLeft);
    addChild("Back Right", backRight);
    addChild("Back Left", backLeft);
    builder.addDoubleProperty("Filtered pitch deg", this::getFilteredPitch, null);
    builder.addDoubleProperty(
        "Throttle multiplier",
        () -> {
          return throttleMultiplier;
        },
        null);
    builder.addDoubleProperty(
        "Target Heading (deg)",
        () -> {
          return targetHeadingDegrees;
        },
        null);
    builder.addDoubleProperty("Gyro Yaw (deg)", gyro::getYaw, null);
    builder.addDoubleProperty("Odometry X (m)", () -> getPose().getX(), null);
    builder.addDoubleProperty("Odometry Y (m)", () -> getPose().getY(), null);
    builder.addDoubleProperty(
        "Odometry Yaw (deg)", () -> getPose().getRotation().getDegrees(), null);
    builder.addDoubleProperty(
          "Front Left Rel Encoder", () -> {return frontLeft.turningSparkMax.getEncoder().getPosition();}, null);  
    builder.addDoubleProperty(
        "Front Left Abs Encoder (rad)", frontLeft::getTurningEncoderAbsPositionRad, null);
    builder.addDoubleProperty(
        "Front Right Abs Encoder (rad)", frontRight::getTurningEncoderAbsPositionRad, null);
    builder.addDoubleProperty(
        "Back Left Abs Encoder (rad)", backLeft::getTurningEncoderAbsPositionRad, null);
    builder.addDoubleProperty(
        "Back Right Abs Encoder (rad)", backRight::getTurningEncoderAbsPositionRad, null);
    builder.addDoubleProperty(
        "Front Left Module Pos (rad)", () -> frontLeft.getPosition().angle.getRadians(), null);
    builder.addDoubleProperty(
        "Front Right Module Pos (rad)", () -> frontRight.getPosition().angle.getRadians(), null);
    builder.addDoubleProperty(
        "Back Left Module Pos (rad)", () -> backLeft.getPosition().angle.getRadians(), null);
    builder.addDoubleProperty(
        "Back Right Module Pos (rad)", () -> backRight.getPosition().angle.getRadians(), null);
    builder.addDoubleProperty(
        "Front Left Distance (m)", () -> frontLeft.getPosition().distanceMeters, null);
    builder.addDoubleProperty(
        "Front Right Distance (m)", () -> frontRight.getPosition().distanceMeters, null);
    builder.addDoubleProperty(
        "Back Left Distance (m)", () -> backLeft.getPosition().distanceMeters, null);
    builder.addDoubleProperty(
        "Back Right Distance (m)", () -> backRight.getPosition().distanceMeters, null);
  }
}
