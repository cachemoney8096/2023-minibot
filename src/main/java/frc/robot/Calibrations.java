package frc.robot;

public final class Calibrations {
    public static final int PLACEHOLDER_INT = 0;
    public static final double PLACEHOLDER_DOUBLE = 0.0;
    public static final float PLACEHOLDER_FLOAT = 0;


    public static final int SPARK_INIT_RETRY_ATTEMPTS = PLACEHOLDER_INT;

    public static final class SwerveModule {
        public static final double DRIVING_P = PLACEHOLDER_DOUBLE,
            DRIVING_I = PLACEHOLDER_DOUBLE,
            DRIVING_D = PLACEHOLDER_DOUBLE,
            DRIVING_FF = PLACEHOLDER_DOUBLE;
        public static final double TURNING_P = PLACEHOLDER_DOUBLE,
            TURNING_I = PLACEHOLDER_DOUBLE,
            TURNING_D = PLACEHOLDER_DOUBLE,
            TURNING_FF = PLACEHOLDER_DOUBLE;
        public static final double DRIVING_MIN_OUPTUT = -1.0, 
            DRIVING_MAX_OUTPUT = 1.0; 
        public static final double TURNING_MIN_OUTPUT = -1.0, 
            TURNING_MAX_OUTPUT = 1.0;        
    } 
}
