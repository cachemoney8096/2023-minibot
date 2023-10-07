package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.grabber.Grabber;

public class CancelScore extends SequentialCommandGroup {
  public CancelScore(Arm arm, Grabber grabber, Lights lights) {
    addRequirements(arm, grabber, lights);
    addCommands(
<<<<<<< HEAD
        new InstantCommand(() -> arm.cancelScore()),
        new InstantCommand(() -> grabber.stopMotors()),
        new InstantCommand(() -> lights.toggleCode(Lights.LightCode.OFF)));
=======
        new InstantCommand(()-> arm.cancelScore()),
        new InstantCommand(() -> grabber.stopMotors()),
        new InstantCommand(() -> lights.toggleCode(Lights.LightCode.OFF))
    );
>>>>>>> 910100179eaee45c8106da5f517c31121b40dba5
  }
}
