package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.grabber.GrabberCalibrations;
import frc.robot.utils.ScoringLocationUtil.ScoreHeight;

public class FinishScore extends SequentialCommandGroup {
  public FinishScore(Arm arm, Grabber grabber, Lights lights) {
    ScoreHeight height = arm.getScoreHeight();
    double scoringPower;
    if (height == ScoreHeight.LOW) {
      scoringPower = GrabberCalibrations.SCORE_LOW_POWER;
    } else if (height == ScoreHeight.MID) {
      scoringPower = GrabberCalibrations.SCORE_MID_POWER;
    } else {
      scoringPower = GrabberCalibrations.SCORE_HIGH_POWER;
    }

    addRequirements(arm, grabber, lights);
    addCommands(
        new InstantCommand(() -> grabber.score(scoringPower)),
        new WaitCommand(GrabberCalibrations.SCORING_WAIT_TIME),
        new InstantCommand(() -> grabber.stopMotors()),
        new InstantCommand(() -> arm.goToPosition(ArmPosition.STARTING)),
        new InstantCommand(() -> lights.toggleCode(Lights.LightCode.OFF)));
  }
}
