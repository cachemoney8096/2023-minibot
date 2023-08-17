package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
    public enum LightCode{
        GAME_OBJECT, // green
        READY_TO_SCORE, // blue
        NO_TAG, // red
        ALIGNING_TO_TAG, // yellow
        PARTY_MODE, // rainbow 🌈
        DEFAULT, // green back and forth (larson animation)
        OFF
    }
    public Lights(){}
    @Override
    public void periodic(){}
    public void setLEDs(){}
    public void setStatusFramePeriod(){}
}
