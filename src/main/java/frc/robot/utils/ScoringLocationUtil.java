package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

// this code is pretty much copied from the 2023 charged up robot (Piggie Smalls) code
public class ScoringLocationUtil {

  /** Height to score at */
  public enum ScoreHeight {
    LOW,
    MID,
    HIGH
  }

  /** Column to score at */
  public enum ScoreCol {
    LEFT,
    CENTER,
    RIGHT
  }

  private ScoreHeight scoreHeight = ScoreHeight.HIGH; // default value
  private ScoreCol scoreCol = ScoreCol.LEFT; // default value
  private boolean middleGrid = false; // default value

  /** Flip the value of middleGrid */
  public void toggleMiddleGrid() {
    middleGrid = !middleGrid;
  }

  // TODO map scoreCol and scoreHeight to tag IDs.
  public void setScoreCol(ScoreCol chooseCol) {
    this.scoreCol = chooseCol;
  }

  public void setScoreHeight(ScoreHeight chooseHeight) {
    this.scoreHeight = chooseHeight;
  }

  public ScoreHeight getScoreHeight() {
    return this.scoreHeight;
  }

  public ScoreCol getScoreCol() {
    return this.scoreCol;
  }

  /** Returns true if the lift is scoring in a high location */
  // TODO I don't know if we need this
  public boolean isScoringHigh() {
    return getScoreHeight() == ScoreHeight.HIGH;
  }

  public Transform2d scoringLocationFromTag() {
    /**
     * From tag space: X+ -> Out of the tag Y+ -> Pointing to the right of the tag (If you are
     * looking at the target) This means positive yaw is based on Z+ being up
     */

    // these values are from the 2023 robot (Piggie Smalls)
    double outwardMeters = -0.470;
    double rightLeftMeters = 0;
    switch (scoreCol) {
      case LEFT:
        rightLeftMeters = -0.556;
        break;
      case RIGHT:
        rightLeftMeters = 0.556;
        break;
      case CENTER:
        rightLeftMeters = 0.0;
        break;
    }
    Translation2d translation = new Translation2d(outwardMeters, rightLeftMeters);
    Rotation2d rot = Rotation2d.fromDegrees(0);
    return new Transform2d(translation, rot);
  }
}
