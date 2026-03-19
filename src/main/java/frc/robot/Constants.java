package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class VisionConstants {
        // Limelight name (as configured in the Limelight web interface)
        public static final String LIMELIGHT_NAME = "limelight";

        // Maximum ambiguity threshold for accepting AprilTag poses
        public static final double MAX_AMBIGUITY = 0.3;

        // Maximum distance to accept vision measurements (meters)
        public static final double MAX_VISION_DISTANCE = 4.0;

        // Minimum number of tags required for pose estimation
        public static final int MIN_TAG_COUNT = 1;

        // Standard deviations for vision measurements [x, y, theta]
        // Lower values = trust vision more, higher values = trust odometry more
        public static final double[] VISION_STD_DEVS = {0.5, 0.5, 0.5};

        // Camera position relative to robot center (meters and degrees)
        // Adjust these based on your camera mounting position
        public static final double CAMERA_FORWARD = 0.3;  // Forward from center
        public static final double CAMERA_SIDE = 0.0;     // Left/right from center
        public static final double CAMERA_UP = 0.5;       // Up from ground
        public static final double CAMERA_ROLL = 0.0;     // Roll angle
        public static final double CAMERA_PITCH = 15.0;   // Pitch angle (tilted up)
        public static final double CAMERA_YAW = 0.0;      // Yaw angle
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
