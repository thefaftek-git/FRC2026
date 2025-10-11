// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;

/**
 * Represents a single swerve module with drive and turning motors.
 */
public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;
  
  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;
  
  private final CANcoder m_canCoder;
  private final double m_encoderOffset;
  
  private final SparkPIDController m_drivePIDController;
  private final SparkPIDController m_turningPIDController;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorId CAN ID for the drive motor
   * @param turningMotorId CAN ID for the turning motor
   * @param canCoderId CAN ID for the CANcoder
   * @param encoderOffset Offset for the CANcoder in radians
   */
  public SwerveModule(
      int driveMotorId,
      int turningMotorId,
      int canCoderId,
      double encoderOffset) {
    
    m_driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
    
    // Factory reset to ensure clean state
    m_driveMotor.restoreFactoryDefaults();
    m_turningMotor.restoreFactoryDefaults();
    
    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getEncoder();
    
    m_canCoder = new CANcoder(canCoderId);
    m_encoderOffset = encoderOffset;
    
    // Configure drive encoder to return meters
    m_driveEncoder.setPositionConversionFactor(1.0 / DriveConstants.kDriveMotorRotationsPerMeter);
    m_driveEncoder.setVelocityConversionFactor(1.0 / DriveConstants.kDriveMotorRotationsPerMeter / 60.0);
    
    // Configure turning encoder to return radians
    m_turningEncoder.setPositionConversionFactor(2 * Math.PI);
    m_turningEncoder.setVelocityConversionFactor(2 * Math.PI / 60.0);
    
    // Get PID controllers
    m_drivePIDController = m_driveMotor.getPIDController();
    m_turningPIDController = m_turningMotor.getPIDController();
    
    // Set PID constants for drive motor
    m_drivePIDController.setP(DriveConstants.kDriveP);
    m_drivePIDController.setI(DriveConstants.kDriveI);
    m_drivePIDController.setD(DriveConstants.kDriveD);
    
    // Set PID constants for turning motor
    m_turningPIDController.setP(DriveConstants.kTurningP);
    m_turningPIDController.setI(DriveConstants.kTurningI);
    m_turningPIDController.setD(DriveConstants.kTurningD);
    
    // Enable PID wrapping for turning motor (continuous input from -pi to pi)
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(-Math.PI);
    m_turningPIDController.setPositionPIDWrappingMaxInput(Math.PI);
    
    // Save configurations
    m_driveMotor.burnFlash();
    m_turningMotor.burnFlash();
    
    // Reset encoders
    resetEncoders();
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(
        desiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Set drive motor velocity
    m_drivePIDController.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    
    // Set turning motor position
    m_turningPIDController.setReference(state.angle.getRadians(), CANSparkMax.ControlType.kPosition);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    
    // Set turning encoder to CANcoder absolute position
    double absolutePosition = m_canCoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
    m_turningEncoder.setPosition(absolutePosition - m_encoderOffset);
  }

  /**
   * Stops the module motors.
   */
  public void stop() {
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }
}
