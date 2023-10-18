package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations;
import frc.robot.commands.autos.components.AutoChargeStationBalance;
import frc.robot.commands.autos.components.AutoMobilityChargeStationSequence;
import frc.robot.commands.autos.components.AutoScoreOne;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.grabber.Grabber;

public class AutoScoreOneAndBalance extends SequentialCommandGroup {
    public static final double DISTANCE_UP_CHARGE_STATION_METERS = 4.5;
    public static final double DISTANCE_BACK_CHARGE_STATION_METERS = 3.15;
    public AutoScoreOneAndBalance(Arm arm, Grabber grabber, Lights lights, DriveSubsystem drive) {
        addCommands(
            new AutoScoreOne(true, arm, grabber, lights),
            /* assume we get mobility in addition to balancing + score 1 */
            new AutoMobilityChargeStationSequence(drive, DISTANCE_UP_CHARGE_STATION_METERS, DISTANCE_BACK_CHARGE_STATION_METERS)
        );
    }
}
