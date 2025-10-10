# FRC2026 - Team 2026 Robot Code

This repository contains the robot code for FIRST Robotics Competition Team 2026's 2025 season robot.

## Robot Configuration
- **Drive System**: Swerve Drive
- **Drivebase Size**: 28" x 28"
- **Framework**: WPILib Command-Based
- **Language**: Java 17

## Project Structure
```
FRC2026/
├── src/main/java/frc/robot/     # Robot source code
│   ├── Main.java                 # Robot main entry point
│   ├── Robot.java                # Main robot class
│   ├── RobotContainer.java       # Robot subsystems and commands
│   └── Constants.java            # Robot constants
├── src/main/deploy/              # Deploy files (configs, paths, etc.)
├── vendordeps/                   # Vendor library dependencies
├── build.gradle                  # Gradle build configuration
└── .github/workflows/            # CI/CD workflows
```

## Getting Started

### Prerequisites
- [WPILib 2025](https://docs.wpilib.org/en/latest/docs/zero-to-robot/step-2/wpilib-setup.html)
- Java 17 (included with WPILib)
- Git

### Building the Project
```bash
./gradlew build
```

### Deploying to Robot
```bash
./gradlew deploy
```

### Running Tests
```bash
./gradlew test
```

## Development

### Opening in VS Code
1. Open VS Code
2. Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on macOS)
3. Type "WPILib: Open Folder" and select this project folder

### CI/CD
This project uses GitHub Actions for continuous integration. Every push and pull request will:
- Build the robot code
- Run all tests
- Archive build artifacts

The workflow status can be viewed in the "Actions" tab of this repository.

## Resources
- [WPILib Documentation](https://docs.wpilib.org/)
- [FIRST Robotics Competition](https://www.firstinspires.org/robotics/frc)
- [Chief Delphi Forums](https://www.chiefdelphi.com/)

## License
This project is licensed under the WPILib BSD License.
