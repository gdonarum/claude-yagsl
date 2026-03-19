# YAGSL Swerve Drive Project

FRC robot project using WPILib 2026 and [YAGSL](https://yagsl.com/) (Yet Another Generic Swerve Library) for swerve drive control.

## Requirements

- [WPILib 2026](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
- FRC Game Tools 2026

## Quick Start

1. Clone the repository:
   ```bash
   git clone git@github.com:gdonarum/claude-yagsl.git
   ```

2. Open in WPILib VS Code

3. Build the project:
   ```bash
   ./gradlew build
   ```

4. Deploy to robot:
   ```bash
   ./gradlew deploy
   ```

## Project Structure

```
├── src/main/java/frc/robot/
│   ├── Robot.java              # Main robot class
│   ├── RobotContainer.java     # Command bindings and subsystems
│   ├── Constants.java          # Robot constants
│   └── subsystems/
│       └── SwerveSubsystem.java
├── src/main/deploy/swerve/     # YAGSL configuration files
│   ├── swervedrive.json
│   ├── controllerproperties.json
│   └── modules/
│       ├── frontleft.json
│       ├── frontright.json
│       ├── backleft.json
│       ├── backright.json
│       ├── physicalproperties.json
│       └── pidfproperties.json
└── vendordeps/                 # Vendor dependencies
```

## Vendor Dependencies

| Library | Version | Purpose |
|---------|---------|---------|
| YAGSL | 2026.3.14 | Swerve drive library |
| Phoenix6 | 26.1.2 | CTRE motor controllers |
| REVLib | 2026.0.5 | REV Robotics hardware |
| Studica | 2026.0.0 | NavX gyroscope |
| ReduxLib | 2026.1.1 | Redux sensors |
| ThriftyLib | 2026.1.2 | ThriftyBot hardware |

## Controls

| Button | Action |
|--------|--------|
| Left Stick | Drive (X/Y translation) |
| Right Stick X | Rotation |
| Start | Zero gyroscope |
| X | Lock wheels |

## Simulation

Run the robot in simulation:
```bash
./gradlew simulateJava
```

## Configuration

Swerve drive configuration is done through JSON files in `src/main/deploy/swerve/`. See the [YAGSL documentation](https://docs.yagsl.com/) for configuration details.

## License

This project is open source and available under the MIT License.
