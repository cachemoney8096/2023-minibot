package frc.robot.commands.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autos.components.AutoChargeStationBalance;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DriveCal;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.grabber.Grabber;

/** scores preload, gets another game piece and scores it, balances (OPEN SIDE) */
public class AutoScoreTwoAndBalance extends SequentialCommandGroup {
  private PathPlannerTrajectory trajBalance =
      PathPlanner.loadPath(
          "BalanceNoBump",
          new PathConstraints(
              DriveCal.MAX_LINEAR_SPEED_METERS_PER_SEC,
              DriveCal.MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));

  public AutoScoreTwoAndBalance(
      boolean red, boolean fast, DriveSubsystem drive, Arm arm, Grabber grabber, Lights lights) {

    if (red) {
      trajBalance =
          PathPlannerTrajectory.transformTrajectoryForAlliance(
              trajBalance, DriverStation.Alliance.Red);
    }

    addRequirements(drive, arm, grabber, lights);

    addCommands(
        new AutoScoreTwo(red, fast, drive, arm, grabber, lights),
        drive.followTrajectoryCommand(trajBalance, false),
        new AutoChargeStationBalance(drive));
  }
}
