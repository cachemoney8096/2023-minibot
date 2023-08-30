package frc.robot.subsystems;

import java.util.TreeMap;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Lights extends SubsystemBase {
  private TreeMap<LightCode, Integer[]> lightOptionsMap;
  private CANdle candle = new CANdle(RobotMap.CANDLE_CAN_ID);
  private CANdleConfiguration config = new CANdleConfiguration();
  private LightCode currentLightStatus = LightCode.OFF;
  public enum LightCode {
    GAME_OBJECT, // green
    READY_TO_SCORE, // blue
    NO_TAG, // red
    ALIGNING_TO_TAG, // yellow
    PARTY_MODE, // rainbow 🌈
    DEFAULT, // green back and forth (larson animation)
    OFF
  }

  public Lights() {

    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 1.0; 
    candle.configAllSettings(config);

    lightOptionsMap = new TreeMap<LightCode, Integer[]>();
    lightOptionsMap.put(LightCode.GAME_OBJECT, new Integer[]{0, 0, 255});
    lightOptionsMap.put(LightCode.READY_TO_SCORE, new Integer[]{0, 255, 0});
    lightOptionsMap.put(LightCode.NO_TAG, new Integer[]{255, 0, 0});
    lightOptionsMap.put(LightCode.ALIGNING_TO_TAG, new Integer[]{255, 255, 0});
    lightOptionsMap.put(LightCode.OFF, new Integer[]{0,0,0});
  }

  public void toggleCode(LightCode light) {
    if (currentLightStatus == light) {
      currentLightStatus = LightCode.OFF;
    } else {
      currentLightStatus = light;
    }
  }
  public void setLEDs(LightCode light) {
    currentLightStatus = light;
    candle.setLEDs(
      lightOptionsMap.get(currentLightStatus)[0], 
      lightOptionsMap.get(currentLightStatus)[1], 
      lightOptionsMap.get(currentLightStatus)[2]
    );
  }
  public void setPartyMode() {
    // party code, needs to use RainbowAnimation and candle.animate()
  }

  public void setStatusFramePeriod() {}

  @Override
  public void periodic() {}
}
