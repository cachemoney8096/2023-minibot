package frc.robot.utils;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;

import java.util.function.BooleanSupplier;

public class SparkMaxUtils {

  /**
   * @param error API return value
   * @return
   */
  public static int check(REVLibError error) {
    return error == REVLibError.kOk ? 0 : 1;
  }

  public static class UnitConversions {
    public static void setRadsFromGearRatio(AbsoluteEncoder sparkMaxEncoder, double ratio) {
      double degreesPerRotation = (2.0 * Math.PI) / ratio;
      double degreesPerRotationPerSecond = degreesPerRotation / 60.0;
      sparkMaxEncoder.setPositionConversionFactor(degreesPerRotation);
      sparkMaxEncoder.setVelocityConversionFactor(degreesPerRotationPerSecond);
    }
  }

  public static void initWithRetry(BooleanSupplier initFunction, int maxRetryAttempts) {
    int numAttempts = 0;
    while (!initFunction.getAsBoolean()) {
      numAttempts++;
      if (numAttempts > maxRetryAttempts) {
        break;
      }
    }
  }
}
