package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
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

        // Publish telemetry data
        Pose2d pose = getPose();
        SmartDashboard.putNumber("Robot X", pose.getX());
        SmartDashboard.putNumber("Robot Y", pose.getY());
        SmartDashboard.putNumber("Robot Heading", pose.getRotation().getDegrees());
    }
}
