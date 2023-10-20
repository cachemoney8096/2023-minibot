package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.LightCode;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.grabber.Grabber;

public class IntakeSequence extends SequentialCommandGroup {
  public IntakeSequence(Arm arm, Grabber grabber, Lights lights) {
    addRequirements(arm, grabber, lights);
    addCommands(
        new InstantCommand(() -> lights.toggleCode(LightCode.OFF)),
        new InstantCommand(() -> arm.goToPosition(ArmPosition.INTAKE)),
        new WaitUntilCommand(arm::atDesiredArmPosition),
        new InstantCommand(() -> grabber.intake()),
        new WaitUntilCommand((grabber::seeGamePiece)),
        new InstantCommand(() -> lights.toggleCode(LightCode.GAME_OBJECT)),
        new InstantCommand(() -> grabber.stopMotors()));
  }

  public static Command interruptibleIntakeSequence(Arm arm, Grabber grabber, Lights lights) {
    return new IntakeSequence(arm, grabber, lights)
        .finallyDo(
            (boolean interrupted) -> {
              grabber.stopMotors();
              lights.toggleCode(LightCode.OFF);
            });
  }
}
