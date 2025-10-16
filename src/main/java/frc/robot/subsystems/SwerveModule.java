// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.DriveConstants;

/**
 * Represents a single swerve module with drive and turning motors.
 */
public class SwerveModule {
  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;
  
  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;
  
  private final CANcoder m_canCoder;
  private final double m_encoderOffset;
  
  private final SparkClosedLoopController m_drivePIDController;
  private final SparkClosedLoopController m_turningPIDController;

  // Simulation objects
  private final SparkMaxSim m_driveMotorSim;
  private final SparkMaxSim m_turningMotorSim;
  private final CANcoderSimState m_canCoderSim;

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
    
    m_driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
    
    // Create configurations for drive and turning motors
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    SparkMaxConfig turningConfig = new SparkMaxConfig();
    
    // Configure drive motor
    driveConfig.encoder
        .positionConversionFactor(1.0 / DriveConstants.kDriveMotorRotationsPerMeter)
        .velocityConversionFactor(1.0 / DriveConstants.kDriveMotorRotationsPerMeter / 60.0);
    driveConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(DriveConstants.kDriveP)
        .i(DriveConstants.kDriveI)
        .d(DriveConstants.kDriveD);
    
    // Configure turning motor
    turningConfig.encoder
        .positionConversionFactor(2 * Math.PI)
        .velocityConversionFactor(2 * Math.PI / 60.0);
    turningConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(DriveConstants.kTurningP)
        .i(DriveConstants.kTurningI)
        .d(DriveConstants.kTurningD)
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(-Math.PI)
        .positionWrappingMaxInput(Math.PI);
    
    // Apply configurations
    m_driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getEncoder();
    
    m_canCoder = new CANcoder(canCoderId);
    m_encoderOffset = encoderOffset;
    
    // Get PID controllers
    m_drivePIDController = m_driveMotor.getClosedLoopController();
    m_turningPIDController = m_turningMotor.getClosedLoopController();
    
    // Initialize simulation objects
    if (RobotBase.isSimulation()) {
      m_driveMotorSim = new SparkMaxSim(m_driveMotor, DCMotor.getNEO(1));
      m_turningMotorSim = new SparkMaxSim(m_turningMotor, DCMotor.getNEO(1));
      m_canCoderSim = m_canCoder.getSimState();
    } else {
      m_driveMotorSim = null;
      m_turningMotorSim = null;
      m_canCoderSim = null;
    }
    
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
    m_drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    
    // Set turning motor position
    m_turningPIDController.setReference(state.angle.getRadians(), ControlType.kPosition);
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

  /**
   * Updates the simulation state of the swerve module.
   * This method should be called periodically in simulation mode.
   */
  public void simulationPeriodic() {
    if (RobotBase.isSimulation()) {
      // Update drive motor simulation
      m_driveMotorSim.iterate(m_driveEncoder.getVelocity(), 12.0, 0.02);
      
      // Update turning motor simulation
      m_turningMotorSim.iterate(m_turningEncoder.getVelocity(), 12.0, 0.02);
      
      // Update CANcoder simulation with current turning position
      m_canCoderSim.setRawPosition(m_turningEncoder.getPosition() / (2 * Math.PI));
      m_canCoderSim.setVelocity(m_turningEncoder.getVelocity() / (2 * Math.PI));
    }
  }
}
