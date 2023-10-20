package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import java.util.TreeMap;

public class Lights extends SubsystemBase {
  private TreeMap<LightCode, Integer[]> lightOptionsMap;
  private CANdle candle = new CANdle(RobotMap.CANDLE_CAN_ID);
  private CANdleConfiguration config = new CANdleConfiguration();
  private LightCode currentLightStatus = LightCode.OFF;

  public enum LightCode {
    GAME_OBJECT, // green
    READY_TO_SCORE, // blue
    NO_TAG, // blinking green animation
    ALIGNING_TO_TAG, // yellow
    PARTY_MODE, // rainbow 🌈
    OFF
  };

  public Lights() {

    // config.stripType = LEDStripType.RGB;
    // config.brightnessScalar = 1.0;
    // candle.configAllSettings(config);

    // // array of integers that represents the R, G, B values of each lightcode enum
    // lightOptionsMap = new TreeMap<LightCode, Integer[]>();
    // lightOptionsMap.put(LightCode.GAME_OBJECT, new Integer[] {0, 255, 0});
    // lightOptionsMap.put(LightCode.READY_TO_SCORE, new Integer[] {0, 0, 255});
    // lightOptionsMap.put(LightCode.ALIGNING_TO_TAG, new Integer[] {255, 255, 0});
    // lightOptionsMap.put(LightCode.OFF, new Integer[] {0, 0, 0});
  }

  public void toggleCode(LightCode light) {
    // if (currentLightStatus == light) {
    //   currentLightStatus = LightCode.OFF;
    // } else {
    //   currentLightStatus = light;
    // }
    // setLEDs();
  }

  private void setLEDs() {
    // if (currentLightStatus == LightCode.PARTY_MODE) {
    //   setPartyMode();
    // } else if (currentLightStatus == LightCode.NO_TAG) {
    //   setNoTag();
    // } else {
    //   candle.setLEDs(
    //       lightOptionsMap.get(currentLightStatus)[0],
    //       lightOptionsMap.get(currentLightStatus)[1],
    //       lightOptionsMap.get(currentLightStatus)[2]);
    // }
  }

  /** No Tag is an animation that switches between no color and green */
  private void setNoTag() {
    // StrobeAnimation strobeAnim = new StrobeAnimation(0, 255, 0);
    // candle.animate(strobeAnim);
  }
  ;

  /** Party Mode is an animation that switches between all the colors of the rainbow */
  public void setPartyMode() {
    // RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, Constants.NUM_CANDLE_LEDS);
    // candle.animate(rainbowAnim);
  }
  ;

  @Override
  public void periodic() {}
}
