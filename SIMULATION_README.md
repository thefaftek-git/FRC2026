# Robot Simulation Guide

This project includes full simulation support for testing the swerve drive robot code without hardware.

## Running the Simulation

### Using VS Code (Recommended)

1. Open the project in VS Code with the WPILib extension installed
2. Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac) to open the command palette
3. Type "WPILib" and select **"WPILib: Simulate Robot Code"**
4. Select **"Desktop Support"** when prompted
5. The simulation GUI will launch with:
   - Robot visualization
   - Driver Station controls
   - Motor controller GUIs (SPARK MAX controllers)
   - Sensor displays (Pigeon2 gyro, CANcoders)

### Using Gradle Command Line

```bash
# Run simulation with GUI
./gradlew simulateJava

# Run simulation in headless mode (no GUI)
./gradlew simulateJavaDebug
```

## What's Simulated

### Motors
- **Drive Motors**: 4x NEO motors for wheel drive
- **Turning Motors**: 4x NEO motors for module steering
- All motors use REVLib SparkMaxSim for accurate simulation

### Sensors
- **Pigeon2 Gyro**: Simulates robot rotation based on swerve kinematics
- **CANcoders**: Simulates absolute encoders for wheel angles
- **NEO Encoders**: Built-in encoder simulation for position and velocity

### Subsystems
- **SwerveDrive**: Full swerve drive kinematics with 4 modules
- **SwerveModule**: Individual module simulation with position control

## Simulation Features

### Automatic GUI Generation
- Motor controllers appear automatically in the simulation GUI
- Real-time visualization of:
  - Motor output percentages
  - Encoder positions and velocities
  - PID controller states
  - Current draw

### Physics Simulation
- Realistic motor behavior based on NEO specifications
- Closed-loop position control for steering
- Closed-loop velocity control for drive
- Inertia and friction modeling

## Testing Your Changes

1. **Start Simulation**: Follow the steps above to launch the simulator
2. **Enable Robot**: Click "Enable" in the simulated Driver Station
3. **Test Teleop**: 
   - Connect a joystick or use keyboard controls
   - Observe module angles in the GUI
   - Verify drive motors respond to commands
4. **Check Steering**: 
   - Modules should rotate to 0 degrees at startup
   - Steering should be smooth and responsive
   - Check CANcoder displays for accurate angle feedback

## Debugging Tips

### If Steering Doesn't Work
- Check the simulation GUI for motor controller status
- Verify PID constants in `Constants.java`
- Look for error messages in the console
- Check that feedback sensors are configured correctly

### If Motors Don't Move
- Ensure robot is enabled in Driver Station
- Check that joystick is connected
- Verify motor CAN IDs match hardware
- Look for fault indicators in motor controller GUIs

## Simulation vs Hardware

### What's the Same
- Code logic and control flow
- PID tuning (approximate)
- Command scheduling
- Sensor reading patterns

### What's Different
- No real physics or inertia (simplified model)
- Perfect sensors (no noise)
- Instant motor response (no electrical delays)
- No mechanical wear or backlash

## Additional Resources

- [WPILib Robot Simulation](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html)
- [REV Robotics Simulation Guide](https://docs.revrobotics.com/revlib/spark/sim/simulation-getting-started)
- [CTRE Phoenix 6 Simulation](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/simulation/index.html)

## Troubleshooting

### Simulation Won't Start
```bash
# Clean and rebuild
./gradlew clean
./gradlew build
```

### GUI Doesn't Show Devices
- Devices are created on-demand when referenced in code
- Make sure subsystems are instantiated in `RobotContainer`
- Check console for any initialization errors

### Performance Issues
- Close unnecessary applications
- Reduce simulation update rate if needed
- Use headless mode for automated testing
