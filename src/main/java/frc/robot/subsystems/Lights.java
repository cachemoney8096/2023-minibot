package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import java.util.TreeMap;

public class Lights extends SubsystemBase {
  private TreeMap<LightCode, Integer[]> lightOptionsMap;
  private CANdle candle = new CANdle(RobotMap.CANDLE_CAN_ID);
  private CANdleConfiguration config = new CANdleConfiguration();
  private LightCode currentLightStatus = LightCode.OFF;
  
  /** Timer for the Blinking LED */
  private int blinkingTimer = 0;
  /** When the blinkingTimer reaches 10 (blinkingPeriod), the LED is toggled */
  private int blinkingPeriod = 10;
  /** If LEDs are currently blinking, then True would mean color is currently showing */
  private boolean blinkingColorOn = false;


  public enum LightCode {
    GAME_OBJECT, // green
    READY_TO_SCORE, // blue
    NO_TAG, // green larson animation
    ALIGNING_TO_TAG, // yellow
    PARTY_MODE, // rainbow 🌈
    OFF
  };

  public Lights() {

    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 1.0;
    candle.configAllSettings(config);

    // array of integers that represents the R, G, B values of each lightcode enum
    lightOptionsMap = new TreeMap<LightCode, Integer[]>();
    lightOptionsMap.put(LightCode.GAME_OBJECT, new Integer[] {0, 255, 0});
    lightOptionsMap.put(LightCode.READY_TO_SCORE, new Integer[] {0, 0, 255});
    lightOptionsMap.put(LightCode.ALIGNING_TO_TAG, new Integer[] {255, 255, 0});
    lightOptionsMap.put(LightCode.OFF, new Integer[] {0, 0, 0});
  }

  public void toggleCode(LightCode light) {
    if (currentLightStatus == light) {
      currentLightStatus = LightCode.OFF;
    } else {
      currentLightStatus = light;
    }
    setLEDs();
  }

  public void setLEDs() {
    if (currentLightStatus != LightCode.PARTY_MODE) {
      candle.setLEDs(
        lightOptionsMap.get(currentLightStatus)[0],
        lightOptionsMap.get(currentLightStatus)[1],
        lightOptionsMap.get(currentLightStatus)[2]
      );
    } else {
      setPartyMode();
    }
  }

  /** Party Mode is an animation that switches between all the colors of the rainbow */
  public void setPartyMode() {
    RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, Constants.NUM_CANDLE_LEDS);
    candle.animate(rainbowAnim);
  }

  /**  */
  @Override
  public void periodic() {
    if (currentLightStatus == LightCode.NO_TAG) {
      if (blinkingTimer >= blinkingPeriod) {
        if (blinkingColorOn) {
          candle.setLEDs(0, 255, 0);
        } else {
          candle.setLEDs(0, 0, 0);
        }
        blinkingColorOn = !blinkingColorOn;
        blinkingTimer = 0;
      }
      blinkingTimer++;
    } else {
      blinkingTimer = 0;
    }
  }
}
