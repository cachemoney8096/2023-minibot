package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ArmCal {
    public static final double PLACEHOLDER_DOUBLE = 0.0;
    
    public static final double ARM_P = PLACEHOLDER_DOUBLE;
    public static final double ARM_I = PLACEHOLDER_DOUBLE;
    public static final double ARM_D = PLACEHOLDER_DOUBLE;
    public static final double ARM_MAX_VELOCITY_DEG_PER_SECOND = PLACEHOLDER_DOUBLE;
    public static final double ARM_MAX_ACCELERATION_DEG_PER_SECOND_SQUARED = PLACEHOLDER_DOUBLE;

    // Arm positions in degrees for the armPositionMap
    public static final double ARM_START_POSITION_DEG = 180.0;
    public static final double ARM_INTAKE_POSITION_DEG = 80.0;
    public static final double ARM_LOW_POSITION_DEG = 270.0;
    public static final double ARM_HIGH_MID_POSITION_DEG = 210.0;

    public static final double ARM_AVOID_LIMELIGHT_POSITION_DEG = 250.0;

    /** Absolute encoder position when the arm is at 0 degrees */
    public static double ARM_ABSOLUTE_ENCODER_ZERO_POS_DEG = PLACEHOLDER_DOUBLE;

    /**
     * Margin for when we consider the arm has reached a position.
     * This is logical (for considering where the arm can go) but not functional (does not stop anything when reached).
     * We apply broader margins for the STARTING position as the arm transits through this position.
     */
    public static final double ARM_MARGIN_DEGREES = PLACEHOLDER_DOUBLE, ARM_START_MARGIN_DEGREES = PLACEHOLDER_DOUBLE;

    /** Input deg/s, output volts. From recalc */
    public static final SimpleMotorFeedforward ARM_FEEDFORWARD =
        new SimpleMotorFeedforward(
            0.0, 1.5 * (2.0 * Math.PI) / 360.0, 0.08 * (2.0 * Math.PI) / 360.0);

    public static final double ARBITRARY_ARM_FEED_FORWARD_VOLTS = PLACEHOLDER_DOUBLE;

}
