package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.LightCode;
import frc.robot.subsystems.TagLimelight;
import frc.robot.subsystems.drive.DriveSubsystem;

public class TeleopDriveToTag extends SequentialCommandGroup {

  public TeleopDriveToTag(TagLimelight limelight, DriveSubsystem drive, Lights lights) {
    addCommands(
        new LookForTag(limelight, drive, lights),
        new SwerveFollowerWrapper(drive).withTimeout(2.0).asProxy(),
        new InstantCommand(
            () -> {
              lights.toggleCode(LightCode.READY_TO_SCORE);
            }));
  }
}