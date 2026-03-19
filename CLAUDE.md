# Claude Code Configuration

## Project Overview
This is an FRC (FIRST Robotics Competition) robot project using WPILib 2026 and YAGSL (Yet Another Generic Swerve Library) for swerve drive control.

## Build System
- **Build Tool**: Gradle with GradleRIO plugin
- **Java Version**: 17
- **WPILib Version**: 2026.2.1

## Build Commands
```bash
# Build the project (use WPILib terminal or set JAVA_HOME to WPILib JDK)
./gradlew build

# Deploy to robot
./gradlew deploy

# Run simulation
./gradlew simulateJava
```

## Project Structure
- `src/main/java/frc/robot/` - Robot code
  - `Robot.java` - Main robot class
  - `RobotContainer.java` - Command bindings and subsystem initialization
  - `subsystems/` - Robot subsystems (e.g., SwerveSubsystem)
  - `commands/` - Custom commands
  - `Constants.java` - Robot constants
- `src/main/deploy/` - Files deployed to the robot (swerve configs, etc.)
- `vendordeps/` - Vendor dependency JSON files

## Key Dependencies
- **YAGSL**: Swerve drive library (version 2026.3.14)
- **Phoenix6**: CTRE motor controllers
- **REVLib**: REV Robotics hardware
- **Studica**: NavX gyroscope (formerly separate NavX library)
- **WPILibNewCommands**: Command-based framework

## Testing
The project uses JUnit 5 for testing:
```bash
./gradlew test
```

## Notes
- Always run builds from the WPILib VS Code environment or ensure JAVA_HOME points to `C:\Users\Public\wpilib\2026\jdk`
- Swerve configuration files are in `src/main/deploy/swerve/`
