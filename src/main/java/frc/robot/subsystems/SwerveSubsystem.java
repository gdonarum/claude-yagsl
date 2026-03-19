package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private final NetworkTable limelightTable;

    public SwerveSubsystem() {
        try {
            File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(SwerveConstants.MAX_SPEED);
        } catch (Exception e) {
            throw new RuntimeException("Failed to create swerve drive", e);
        }

        swerveDrive.setHeadingCorrection(true);
        swerveDrive.setCosineCompensator(false);

        // Get Limelight NetworkTable
        limelightTable = NetworkTableInstance.getDefault().getTable(VisionConstants.LIMELIGHT_NAME);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation, rotation, fieldRelative, false);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        swerveDrive.drive(chassisSpeeds);
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    public void lock() {
        swerveDrive.lockPose();
    }

    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

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
        updateVisionOdometry();

        Pose2d pose = getPose();
        SmartDashboard.putNumber("Robot X", pose.getX());
        SmartDashboard.putNumber("Robot Y", pose.getY());
        SmartDashboard.putNumber("Robot Heading", pose.getRotation().getDegrees());
    }

    private void updateVisionOdometry() {
        // Get botpose from Limelight (WPILib Blue alliance origin)
        double[] botpose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);

        if (botpose.length >= 6) {
            double x = botpose[0];
            double y = botpose[1];
            double yaw = botpose[5];
            double latency = botpose.length > 6 ? botpose[6] : 0;

            // Simple validation
            if (x != 0 || y != 0) {
                Pose2d visionPose = new Pose2d(x, y, Rotation2d.fromDegrees(yaw));
                double timestamp = Timer.getFPGATimestamp() - (latency / 1000.0);

                swerveDrive.addVisionMeasurement(visionPose, timestamp);
            }
        }
    }
}
