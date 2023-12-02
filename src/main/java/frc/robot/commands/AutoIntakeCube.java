package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClawLimelight.ClawLimelight;
import frc.robot.subsystems.ClawLimelight.ClawLimelight.CubeDetection;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.grabber.Grabber;

public class AutoIntakeCube extends SequentialCommandGroup {

  private static final double DISTANCE_IN_FRONT_OF_CUBE_METERS = Units.inchesToMeters(3);
  private Optional<CubeDetection> cubeDetection = Optional.empty();
  private Pose2d inFrontOfCube = new Pose2d();
  private static final double SPEED_AT_CUBE_MPS = 2.0;

  private static Pose2d getPoseInFrontOfCube(DriveSubsystem drive, CubeDetection cube) {
    Rotation2d cubeAngle = Rotation2d.fromDegrees(cube.angleDeg);
    double cubeDistanceMeters = cube.distanceMeters + Units.inchesToMeters(6);
    Pose2d poseAtDetection = drive.getPastPose(cube.latencySec);
    Transform2d toInFrontOfcube =
        new Transform2d(
            new Translation2d(cubeDistanceMeters - DISTANCE_IN_FRONT_OF_CUBE_METERS, 0)
                .rotateBy(cubeAngle),
            cubeAngle);
    return poseAtDetection.plus(toInFrontOfcube);
  }

  public AutoIntakeCube(DriveSubsystem drive, Arm arm, Grabber grabber, Lights lights, ClawLimelight clawLimelight) {
    addRequirements(arm, grabber, lights, clawLimelight);
    addCommands(
        new RunCommand(
                () -> {
                  cubeDetection = clawLimelight.getCubePos();
                  if (!cubeDetection.isPresent()) {
                    return;
                  }
                  inFrontOfCube = getPoseInFrontOfCube(drive, cubeDetection.get());
                })
            .until(() -> cubeDetection.isPresent()),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                    new RunCommand(
                            () -> {
                              drive.rotateOrKeepHeading(1.0, 0.0, 0.0, false, -1);
                            },
                            drive)
                        .withTimeout(0.25),
                    new SwerveToPointWrapper(
                        false, drive, () -> inFrontOfCube, 4.0, SPEED_AT_CUBE_MPS))
                .finallyDo(
                    (boolean interrupted) -> {
                      drive.stopDriving();
                    }),
            IntakeSequence.interruptibleIntakeSequence(arm, grabber, lights)));
  }
}
