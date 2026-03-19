package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;

    public SwerveSubsystem() {
        try {
            File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(SwerveConstants.MAX_SPEED);
        } catch (Exception e) {
            throw new RuntimeException("Failed to create swerve drive", e);
        }

        // Configure heading correction
        swerveDrive.setHeadingCorrection(true);
        swerveDrive.setCosineCompensator(false);

        // Configure Limelight camera pose relative to robot
        LimelightHelpers.setCameraPose_RobotSpace(
            VisionConstants.LIMELIGHT_NAME,
            VisionConstants.CAMERA_FORWARD,
            VisionConstants.CAMERA_SIDE,
            VisionConstants.CAMERA_UP,
            VisionConstants.CAMERA_ROLL,
            VisionConstants.CAMERA_PITCH,
            VisionConstants.CAMERA_YAW
        );
    }

    /**
     * Drive the robot using field-relative or robot-relative controls.
     *
     * @param translation   The translation (x/y) velocities in meters per second.
     * @param rotation      The rotation velocity in radians per second.
     * @param fieldRelative Whether the values are field-relative.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation, rotation, fieldRelative, false);
    }

    /**
     * Drive the robot using ChassisSpeeds.
     *
     * @param chassisSpeeds The desired chassis speeds.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        swerveDrive.drive(chassisSpeeds);
    }

    /**
     * Get the current pose of the robot.
     *
     * @return The current pose.
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Reset the odometry to a given pose.
     *
     * @param pose The pose to reset to.
     */
    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    /**
     * Get the current heading of the robot.
     *
     * @return The current heading as a Rotation2d.
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Zero the gyroscope heading.
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    /**
     * Set the motors to brake mode.
     *
     * @param brake True for brake mode, false for coast mode.
     */
    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    /**
     * Get the underlying SwerveDrive object.
     *
     * @return The SwerveDrive object.
     */
    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    /**
     * Lock the swerve modules to prevent movement (X pattern).
     */
    public void lock() {
        swerveDrive.lockPose();
    }

    /**
     * Get the current field-relative velocity.
     *
     * @return The current field-relative ChassisSpeeds.
     */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Get the current robot-relative velocity.
     *
     * @return The current robot-relative ChassisSpeeds.
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Command to drive the swerve drive using field-relative controls.
     *
     * @param vX        Supplier for the X velocity.
     * @param vY        Supplier for the Y velocity.
     * @param omega     Supplier for the angular velocity.
     * @return A command that drives the swerve drive.
     */
    public Command driveCommand(java.util.function.DoubleSupplier vX,
                                 java.util.function.DoubleSupplier vY,
                                 java.util.function.DoubleSupplier omega) {
        return run(() -> {
            double xVelocity = vX.getAsDouble() * SwerveConstants.MAX_SPEED;
            double yVelocity = vY.getAsDouble() * SwerveConstants.MAX_SPEED;
            double angularVelocity = omega.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity();

            drive(new Translation2d(xVelocity, yVelocity), angularVelocity, true);
        });
    }

    @Override
    public void periodic() {
        // Update odometry
        swerveDrive.updateOdometry();

        // Update vision-based odometry
        updateVisionOdometry();

        // Publish telemetry data
        Pose2d pose = getPose();
        SmartDashboard.putNumber("Robot X", pose.getX());
        SmartDashboard.putNumber("Robot Y", pose.getY());
        SmartDashboard.putNumber("Robot Heading", pose.getRotation().getDegrees());
    }

    /**
     * Update odometry using Limelight AprilTag vision measurements.
     * Uses MegaTag2 for more accurate pose estimation.
     */
    private void updateVisionOdometry() {
        // Set robot orientation for MegaTag2 algorithm
        LimelightHelpers.SetRobotOrientation(
            VisionConstants.LIMELIGHT_NAME,
            getHeading().getDegrees(),
            0, // yaw rate (not needed)
            0, // pitch
            0, // pitch rate
            0, // roll
            0  // roll rate
        );

        // Get pose estimate based on alliance
        LimelightHelpers.PoseEstimate poseEstimate;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(VisionConstants.LIMELIGHT_NAME);
        } else {
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LIMELIGHT_NAME);
        }

        // Validate and add vision measurement
        if (isValidVisionMeasurement(poseEstimate)) {
            swerveDrive.addVisionMeasurement(
                poseEstimate.pose,
                poseEstimate.timestampSeconds
            );

            SmartDashboard.putNumber("Vision Tag Count", poseEstimate.tagCount);
            SmartDashboard.putNumber("Vision Avg Distance", poseEstimate.avgTagDist);
        }
    }

    /**
     * Check if a vision pose estimate is valid and should be used.
     */
    private boolean isValidVisionMeasurement(LimelightHelpers.PoseEstimate estimate) {
        if (estimate == null || estimate.tagCount < VisionConstants.MIN_TAG_COUNT) {
            return false;
        }

        // Check if any tags have high ambiguity
        if (estimate.rawFiducials != null) {
            for (var fiducial : estimate.rawFiducials) {
                if (fiducial.ambiguity > VisionConstants.MAX_AMBIGUITY) {
                    return false;
                }
            }
        }

        // Check if average distance is within acceptable range
        if (estimate.avgTagDist > VisionConstants.MAX_VISION_DISTANCE) {
            return false;
        }

        // Check for reasonable pose (not at origin, not wildly off-field)
        Pose2d pose = estimate.pose;
        if (pose.getX() < -1 || pose.getX() > 17 || pose.getY() < -1 || pose.getY() > 9) {
            return false;
        }

        return true;
    }
}
