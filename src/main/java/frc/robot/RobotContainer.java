// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SwerveDrive m_swerveDrive = new SwerveDrive();

  // The driver's controller
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  // Keyboard input for simulation/backup
  private final GenericHID m_keyboard = new GenericHID(OperatorConstants.kKeyboardPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    
    // Configure default commands
    configureDefaultCommands();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Reset gyro heading when back button is pressed
    m_driverController.back().onTrue(Commands.runOnce(() -> m_swerveDrive.zeroHeading(), m_swerveDrive));
    
    // Reset encoders when start button is pressed
    m_driverController.start().onTrue(Commands.runOnce(() -> m_swerveDrive.resetEncoders(), m_swerveDrive));
  }

  /**
   * Configure default commands for subsystems.
   */
  private void configureDefaultCommands() {
    // Set the default command for the drive subsystem
    // Use keyboard if no controller is detected, otherwise use controller
    m_swerveDrive.setDefaultCommand(
        new DriveCommand(
            m_swerveDrive,
            () -> isControllerConnected() ? applyDeadband(-m_driverController.getLeftY()) : getKeyboardForward(),
            () -> isControllerConnected() ? applyDeadband(-m_driverController.getLeftX()) : getKeyboardStrafe(),
            () -> isControllerConnected() ? applyDeadband(-m_driverController.getRightX()) : getKeyboardRotation(),
            true)); // Field-relative drive
  }
  
  /**
   * Check if a controller is connected.
   *
   * @return true if controller is connected
   */
  private boolean isControllerConnected() {
    return DriverStation.isJoystickConnected(OperatorConstants.kDriverControllerPort);
  }
  
  /**
   * Get forward/backward speed from WASD keyboard.
   * W = forward, S = backward
   *
   * @return speed value between -1 and 1
   */
  private double getKeyboardForward() {
    double forward = 0;
    if (m_keyboard.getRawButton(OperatorConstants.kKeyW)) {
      forward += OperatorConstants.kKeyboardSpeed;
    }
    if (m_keyboard.getRawButton(OperatorConstants.kKeyS)) {
      forward -= OperatorConstants.kKeyboardSpeed;
    }
    return forward;
  }
  
  /**
   * Get strafe left/right speed from WASD keyboard.
   * A = left, D = right
   *
   * @return speed value between -1 and 1
   */
  private double getKeyboardStrafe() {
    double strafe = 0;
    if (m_keyboard.getRawButton(OperatorConstants.kKeyA)) {
      strafe -= OperatorConstants.kKeyboardSpeed;
    }
    if (m_keyboard.getRawButton(OperatorConstants.kKeyD)) {
      strafe += OperatorConstants.kKeyboardSpeed;
    }
    return strafe;
  }
  
  /**
   * Get rotation speed from keyboard.
   * Q = counter-clockwise, E = clockwise
   *
   * @return rotation value between -1 and 1
   */
  private double getKeyboardRotation() {
    double rotation = 0;
    if (m_keyboard.getRawButton(OperatorConstants.kKeyQ)) {
      rotation += OperatorConstants.kKeyboardSpeed;
    }
    if (m_keyboard.getRawButton(OperatorConstants.kKeyE)) {
      rotation -= OperatorConstants.kKeyboardSpeed;
    }
    return rotation;
  }

  /**
   * Apply deadband to joystick input.
   *
   * @param value The input value
   * @return The value with deadband applied
   */
  private double applyDeadband(double value) {
    if (Math.abs(value) < OperatorConstants.kDeadband) {
      return 0;
    }
    return value;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Commands.print("No autonomous command configured");
  }
}
