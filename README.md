# FRC2026 - Team 2026 Robot Code

This repository contains the robot code for FIRST Robotics Competition Team 2026's robot.

## Robot Configuration
- **Drive System**: Swerve Drive (MK4n modules with 7.13:1 gear ratio)
- **Drivebase Size**: 28" x 28"
- **Framework**: WPILib Command-Based
- **Language**: Java 17
- **Gyroscope**: Pigeon 2.0
- **Absolute Encoders**: CANcoder on each wheel

## Project Structure
```
FRC2026/
├── src/main/java/frc/robot/
│   ├── Main.java                    # Robot main entry point
│   ├── Robot.java                   # Main robot class
│   ├── RobotContainer.java          # Robot subsystems and commands
│   ├── Constants.java               # Robot constants (CAN IDs, PID values, etc.)
│   ├── subsystems/
│   │   ├── SwerveDrive.java         # Swerve drive subsystem
│   │   └── SwerveModule.java        # Individual swerve module control
│   └── commands/
│       └── DriveCommand.java        # Teleop drive command
├── src/main/deploy/                 # Deploy files (configs, paths, etc.)
├── vendordeps/                      # Vendor library dependencies
├── build.gradle                     # Gradle build configuration
└── .github/workflows/               # CI/CD workflows
```

## Getting Started

### Prerequisites
- [WPILib 2024](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
- Java 17 (included with WPILib)
- Git

### Vendor Dependencies
This project includes the following vendor libraries:
- **REVLib**: For REV Robotics motor controllers (SparkMax, SparkFlex) - used for drive and turning motors
- **Phoenix 6**: For CTRE hardware (CANcoder absolute encoders, Pigeon 2.0 gyroscope)
- **NavX**: For Kauai Labs gyroscope/IMU

## Swerve Drive Setup

### Hardware Configuration
The robot uses a swerve drive system with:
- **4 MK4n swerve modules** with 7.13:1 gear ratio
- **CANcoder absolute encoders** on each wheel for precise angle measurement
- **Pigeon 2.0 gyroscope** for robot heading
- **REV SparkMax motor controllers** for drive and turning motors

### Controller Mapping (Xbox Controller)
- **Left Stick Y-Axis**: Forward/Backward movement
- **Left Stick X-Axis**: Left/Right strafe
- **Right Stick X-Axis**: Rotation
- **Back Button**: Reset gyro heading (field-relative zero)
- **Start Button**: Reset drive encoders

### Initial Calibration
Before the robot can drive properly, you need to:

1. **Set CAN IDs in Constants.java** - Update the motor controller and encoder CAN IDs to match your wiring
2. **Calibrate CANcoder Offsets** - Align all wheels forward and record the absolute positions:
   ```java
   // In Constants.java, update these values:
   kFrontLeftEncoderOffset = <measured_value>;
   kFrontRightEncoderOffset = <measured_value>;
   kBackLeftEncoderOffset = <measured_value>;
   kBackRightEncoderOffset = <measured_value>;
   ```
3. **Tune PID Constants** - Adjust the turning and drive PID values in Constants.java for smooth operation
4. **Verify Maximum Speeds** - Adjust `kMaxSpeedMetersPerSecond` based on your robot's capabilities

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

## Simulation & AdvantageScope Visualization

This project publishes the drivetrain pose to NetworkTables via a `Field2d` instance under the
`SmartDashboard/Field` key. Any tool that understands WPILib field data (including AdvantageScope)
can subscribe to that topic and show the robot driving around the field while you run the WPILib
simulator.

### Steps
1. Start the simulator (`./gradlew simulateJava` or VS Code's "Simulate Robot" command).
2. Launch [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope) and connect to
   `localhost` (or the IP/hostname where the sim is running) using the NT4 data source.
3. Add a **Field** widget pointing at either `SmartDashboard/Field` (straight from WPILib) or
   `RealOutputs/Swerve/Pose` (AdvantageKit pose feed).
4. Drive the robot in sim—the robot pose, module telemetry, and gyro heading will stay in sync with
   what you see on the dashboard, and AdvantageScope can log or replay the `RealOutputs/Swerve/*`
   entries for post-match analysis.

Because the same Field2d data is sent on the real robot, you can also use AdvantageScope for live
events or log playback without changing code.

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
