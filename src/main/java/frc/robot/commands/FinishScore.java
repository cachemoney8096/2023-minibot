package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.grabber.GrabberCalibrations;

public class FinishScore extends SequentialCommandGroup {
  public FinishScore(Arm arm, Grabber grabber, Lights lights) {
    addRequirements(arm, grabber, lights);
    addCommands(
        new InstantCommand(() -> grabber.score(arm.getScoreHeight())),
        new WaitCommand(GrabberCalibrations.SCORING_WAIT_TIME),
        new InstantCommand(() -> grabber.stopMotors()),
        new InstantCommand(() -> arm.goToPosition(ArmPosition.STARTING)),
        new InstantCommand(() -> lights.toggleCode(Lights.LightCode.OFF)));
  }
}
