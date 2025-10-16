// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

/**
 * Command to drive the robot using joystick inputs.
 */
public class DriveCommand extends Command {
  private final SwerveDrive m_swerveDrive;
  private final DoubleSupplier m_xSpeedSupplier;
  private final DoubleSupplier m_ySpeedSupplier;
  private final DoubleSupplier m_rotSupplier;
  private final boolean m_fieldRelative;

  /**
   * Creates a new DriveCommand.
   *
   * @param swerveDrive The swerve drive subsystem
   * @param xSpeedSupplier Supplier for forward/backward speed
   * @param ySpeedSupplier Supplier for left/right speed
   * @param rotSupplier Supplier for rotation speed
   * @param fieldRelative Whether the drive should be field-relative
   */
  public DriveCommand(
      SwerveDrive swerveDrive,
      DoubleSupplier xSpeedSupplier,
      DoubleSupplier ySpeedSupplier,
      DoubleSupplier rotSupplier,
      boolean fieldRelative) {
    m_swerveDrive = swerveDrive;
    m_xSpeedSupplier = xSpeedSupplier;
    m_ySpeedSupplier = ySpeedSupplier;
    m_rotSupplier = rotSupplier;
    m_fieldRelative = fieldRelative;
    
    addRequirements(swerveDrive);
  }

  @Override
  public void execute() {
    // Get joystick values
    double xSpeed = m_xSpeedSupplier.getAsDouble();
    double ySpeed = m_ySpeedSupplier.getAsDouble();
    double rot = m_rotSupplier.getAsDouble();

    // Drive the robot
    m_swerveDrive.drive(xSpeed, ySpeed, rot, m_fieldRelative);
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
