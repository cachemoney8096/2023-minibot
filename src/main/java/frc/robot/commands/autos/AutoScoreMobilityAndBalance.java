package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autos.components.AutoMobilityChargeStationSequence;
import frc.robot.commands.autos.components.AutoScoreOne;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.grabber.Grabber;

/* Scores the initial game piece, drives over (mobility), comes back and balances */
public class AutoScoreMobilityAndBalance extends SequentialCommandGroup {
  public static final double DISTANCE_UP_CHARGE_STATION_METERS = 4.5;
  public static final double DISTANCE_BACK_CHARGE_STATION_METERS = 3.15;

  public AutoScoreMobilityAndBalance(
      Arm arm, Grabber grabber, Lights lights, DriveSubsystem drive) {
    addCommands(
        new AutoScoreOne(true, arm, grabber, lights),
        new AutoMobilityChargeStationSequence(
            drive, DISTANCE_UP_CHARGE_STATION_METERS, DISTANCE_BACK_CHARGE_STATION_METERS));
  }
}
