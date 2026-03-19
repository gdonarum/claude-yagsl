package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class VisionConstants {
        // Limelight name (as configured in the Limelight web interface)
        public static final String LIMELIGHT_NAME = "limelight";
    }

    public static final class SwerveConstants {
        // Maximum speed of the robot in meters per second
        public static final double MAX_SPEED = Units.feetToMeters(14.5);

        // Deadband for joystick inputs
        public static final double DEADBAND = 0.1;

        // Speed multipliers for different drive modes
        public static final double SLOW_MODE_MULTIPLIER = 0.25;
        public static final double NORMAL_MODE_MULTIPLIER = 0.75;
        public static final double FAST_MODE_MULTIPLIER = 1.0;
    }

    public static final class OperatorConstants {
        // Controller ports
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }
}
