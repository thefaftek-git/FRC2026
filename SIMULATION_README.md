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

## Control Options

### Xbox Controller (Primary)
When an Xbox controller is connected, it will be used automatically:
- **Left Stick**: Forward/backward and strafe left/right
- **Right Stick X-axis**: Rotation
- **Back Button**: Reset gyro heading
- **Start Button**: Reset encoders

### WASD Keyboard (Backup/Simulation)
**Keyboard controls are automatically enabled when NO controller is detected:**
- **W**: Drive forward
- **S**: Drive backward
- **A**: Strafe left
- **D**: Strafe right
- **Q**: Rotate counter-clockwise
- **E**: Rotate clockwise

The keyboard provides 60% speed (configurable in Constants.java) for more precise control during testing.

## NetworkTables Telemetry

Real-time swerve module data is published to NetworkTables under `/SwerveModule/`:

### Available Data per Module
- **FrontLeft/AngleDegrees**: Current wheel angle in degrees
- **FrontLeft/VelocityMPS**: Current drive velocity in meters per second
- **FrontRight/AngleDegrees**: Current wheel angle in degrees
- **FrontRight/VelocityMPS**: Current drive velocity in meters per second
- **BackLeft/AngleDegrees**: Current wheel angle in degrees
- **BackLeft/VelocityMPS**: Current drive velocity in meters per second
- **BackRight/AngleDegrees**: Current wheel angle in degrees
- **BackRight/VelocityMPS**: Current drive velocity in meters per second

### Viewing NetworkTables Data

#### Glass (Recommended)
1. Open Glass from WPILib menu or run `glass` from command line
2. Connect to NetworkTables (localhost for simulation, robot IP for real robot)
3. Navigate to NetworkTables view
4. Expand `/SwerveModule/` to see all module data
5. You can create graphs and displays for real-time monitoring

#### Shuffleboard
1. Open Shuffleboard
2. Connect to NetworkTables
3. Drag topics from the left sidebar to create widgets
4. Save layouts for future use

#### OutlineViewer
1. Run `outlineviewer` from WPILib tools
2. Connect to NetworkTables
3. View raw NetworkTables data tree

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
   - If no controller: Use WASD keys for driving
   - If controller connected: Use joysticks normally
   - Observe module angles in NetworkTables/Glass
   - Verify drive motors respond to commands
4. **Check Steering**: 
   - Modules should rotate to 0 degrees at startup
   - Steering should be smooth and responsive
   - Check NetworkTables for accurate angle feedback
5. **Monitor Telemetry**:
   - Open Glass or Shuffleboard
   - Watch wheel angles update in real-time
   - Monitor drive velocities during movement

## Debugging Tips

### If Steering Doesn't Work
- Check the simulation GUI for motor controller status
- Verify PID constants in `Constants.java`
- Look for error messages in the console
- Check NetworkTables to see if angles are updating
- Verify that feedback sensors are configured correctly

### If Motors Don't Move
- Ensure robot is enabled in Driver Station
- Check control input:
  - Controller: Verify joystick is detected
  - Keyboard: Ensure no controller is plugged in
- Verify motor CAN IDs match hardware
- Look for fault indicators in motor controller GUIs

### If Keyboard Doesn't Work
- **Disconnect any controllers** - keyboard only activates when no controller is detected
- Check console for "Joystick connected" messages
- Verify keyboard port is set correctly in Constants.java
- Make sure the simulation window has focus

### NetworkTables Troubleshooting
- Verify NetworkTables server is running (automatic in simulation)
- Check connection in Glass/Shuffleboard
- Look for `/SwerveModule/` table in the tree
- Restart simulation if data isn't updating

## Simulation vs Hardware

### What's the Same
- Code logic and control flow
- PID tuning (approximate)
- Command scheduling
- Sensor reading patterns
- Control input (keyboard or controller)
- NetworkTables telemetry

### What's Different
- No real physics or inertia (simplified model)
- Perfect sensors (no noise)
- Instant motor response (no electrical delays)
- No mechanical wear or backlash
- Keyboard control option (simulation only)

## Additional Resources

- [WPILib Robot Simulation](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html)
- [REV Robotics Simulation Guide](https://docs.revrobotics.com/revlib/spark/sim/simulation-getting-started)
- [CTRE Phoenix 6 Simulation](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/simulation/index.html)
- [NetworkTables Documentation](https://docs.wpilib.org/en/stable/docs/software/networktables/index.html)
- [Glass Telemetry Tool](https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/index.html)

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

### NetworkTables Not Updating
```bash
# Restart the simulation
# Check that periodic() is being called
# Verify publishers were initialized correctly
```
