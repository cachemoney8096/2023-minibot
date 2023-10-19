package frc.robot.commands.autos.components;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.LightCode;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.arm.ArmCal;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.grabber.GrabberCalibrations;
import frc.robot.utils.ScoringLocationUtil.ScoreHeight;

public class AutoScoreOne extends SequentialCommandGroup {
  public AutoScoreOne(boolean fast, Arm arm, Grabber grabber, Lights lights) {
    addCommands(
        new InstantCommand(() -> arm.startScore()),
        new ConditionalCommand(
          new WaitUntilCommand(() -> arm.atPosition(ArmPosition.SCORE_MID_HIGH)).withTimeout(ArmCal.START_TO_PRESCORE_MID_HIGH_SEC),
          new WaitUntilCommand(() -> arm.atPosition(ArmPosition.SCORE_LOW)).withTimeout(ArmCal.START_TO_PRESCORE_LOW_SEC),
            () -> {
              return arm.getScoreHeight() == ScoreHeight.HIGH
                  || arm.getScoreHeight() == ScoreHeight.MID;
            }),
        new InstantCommand(
            () -> {
              lights.toggleCode(LightCode.READY_TO_SCORE);
            }),
        new InstantCommand(() -> grabber.score(arm.getScoreHeight())),
        new WaitCommand(GrabberCalibrations.EJECTION_WAIT_TIME),
        new InstantCommand(() -> arm.goToPosition(ArmPosition.STARTING)),
        new WaitUntilCommand(() -> arm.atPosition(ArmPosition.STARTING))
            .withTimeout(fast ? ArmCal.SCORE_TO_START_FAST_SEC : ArmCal.SCORE_TO_START_SEC));
  }
}
