package frc.robot.commands.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeSequence;
import frc.robot.commands.autos.components.*;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DriveCal;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.grabber.Grabber;
import java.util.HashMap;

public class AutoScoreOneFive extends SequentialCommandGroup {
  private PathPlannerTrajectory oneFive =
      PathPlanner.loadPath(
          "OneFive",
          new PathConstraints(
              DriveCal.MAX_LINEAR_SPEED_METERS_PER_SEC,
              DriveCal.MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ));

  private HashMap<String, Command> eventMap = new HashMap<>();

  public AutoScoreOneFive(DriveSubsystem drive, Arm arm, Grabber grabber, Lights lights) {
    eventMap.put(
        "intakeGamePiece", IntakeSequence.interruptibleIntakeSequence(arm, grabber, lights));
    addCommands(
        new AutoScoreOne(false, arm, grabber, lights),
        new FollowPathWithEvents(
            drive.followTrajectoryCommand(oneFive, true), oneFive.getMarkers(), eventMap));
  }
}
