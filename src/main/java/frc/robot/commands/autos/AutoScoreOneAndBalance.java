package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autos.components.AutoChargeStationBalance;
import frc.robot.commands.autos.components.AutoScoreOne;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.grabber.Grabber;

public class AutoScoreOneAndBalance extends SequentialCommandGroup {
    public AutoScoreOneAndBalance(Arm arm, Grabber grabber, Lights lights, DriveSubsystem drive) {
        addCommands(
            new AutoScoreOne(true, arm, grabber, lights),
            new AutoChargeStationBalance(drive)
        );
    }
}
