package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.LightCode;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.grabber.GrabberCalibrations;

public class OuttakeSequence extends SequentialCommandGroup {
  public OuttakeSequence(Arm arm, Grabber grabber, Lights lights) {
    addRequirements(arm, grabber, lights);
    addCommands(
        new RunCommand(() -> arm.goToPosition(ArmPosition.OUTTAKE))
            .until(() -> arm.atPosition(ArmPosition.OUTTAKE)),
        new InstantCommand(() -> grabber.eject()),
        new WaitCommand(GrabberCalibrations.EJECTION_WAIT_TIME),
        new InstantCommand(() -> grabber.stopMotors()),
        new InstantCommand(() -> lights.toggleCode(LightCode.OFF)));
  }
}
