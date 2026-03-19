package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    // Subsystems
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    // Controllers
    private final CommandXboxController driverController =
        new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Default command for the swerve subsystem - field-relative drive
        swerveSubsystem.setDefaultCommand(
            swerveSubsystem.driveCommand(
                () -> -MathUtil.applyDeadband(driverController.getLeftY(), SwerveConstants.DEADBAND),
                () -> -MathUtil.applyDeadband(driverController.getLeftX(), SwerveConstants.DEADBAND),
                () -> -MathUtil.applyDeadband(driverController.getRightX(), SwerveConstants.DEADBAND)
            )
        );

        // Zero gyro on Start button
        driverController.start().onTrue(Commands.runOnce(swerveSubsystem::zeroGyro));

        // Lock wheels in X pattern on X button
        driverController.x().whileTrue(Commands.run(swerveSubsystem::lock, swerveSubsystem));
    }

    public Command getAutonomousCommand() {
        // Return a simple autonomous command (drive forward for 2 seconds)
        return Commands.run(
            () -> swerveSubsystem.drive(
                new edu.wpi.first.math.geometry.Translation2d(1.0, 0),
                0,
                true
            ),
            swerveSubsystem
        ).withTimeout(2.0);
    }
}
