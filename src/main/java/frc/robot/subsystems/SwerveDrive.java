// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/**
 * Represents a swerve drive style drivetrain.
 */
public class SwerveDrive extends SubsystemBase {
  // Create swerve modules
  private final SwerveModule m_frontLeft = new SwerveModule(
      "FrontLeft",
      DriveConstants.kFrontLeftDriveMotorId,
      DriveConstants.kFrontLeftTurningMotorId,
      DriveConstants.kFrontLeftCanCoderId,
      DriveConstants.kFrontLeftEncoderOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
      "FrontRight",
      DriveConstants.kFrontRightDriveMotorId,
      DriveConstants.kFrontRightTurningMotorId,
      DriveConstants.kFrontRightCanCoderId,
      DriveConstants.kFrontRightEncoderOffset);

  private final SwerveModule m_backLeft = new SwerveModule(
      "BackLeft",
      DriveConstants.kBackLeftDriveMotorId,
      DriveConstants.kBackLeftTurningMotorId,
      DriveConstants.kBackLeftCanCoderId,
      DriveConstants.kBackLeftEncoderOffset);

  private final SwerveModule m_backRight = new SwerveModule(
      "BackRight",
      DriveConstants.kBackRightDriveMotorId,
      DriveConstants.kBackRightTurningMotorId,
      DriveConstants.kBackRightCanCoderId,
      DriveConstants.kBackRightEncoderOffset);

  // The gyro sensor
  private final Pigeon2 m_pigeon = new Pigeon2(DriveConstants.kPigeonId);
  private final Pigeon2SimState m_pigeonSim;
  private double m_simulatedYawRadians = 0.0;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;

  // Locations for the swerve drive modules relative to the robot center
  Translation2d m_frontLeftLocation = new Translation2d(
      DriveConstants.kWheelBase / 2, 
      DriveConstants.kTrackWidth / 2);
  Translation2d m_frontRightLocation = new Translation2d(
      DriveConstants.kWheelBase / 2, 
      -DriveConstants.kTrackWidth / 2);
  Translation2d m_backLeftLocation = new Translation2d(
      -DriveConstants.kWheelBase / 2, 
      DriveConstants.kTrackWidth / 2);
  Translation2d m_backRightLocation = new Translation2d(
      -DriveConstants.kWheelBase / 2, 
      -DriveConstants.kTrackWidth / 2);

  // Creating kinematics object using the module locations
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, 
      m_frontRightLocation, 
      m_backLeftLocation, 
      m_backRightLocation);

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    // Reset gyro
    m_pigeon.reset();

    // Initialize simulation
    if (RobotBase.isSimulation()) {
      m_pigeonSim = m_pigeon.getSimState();
      m_pigeonSim.setSupplyVoltage(DriveConstants.kSimSupplyVoltage);
    } else {
      m_pigeonSim = null;
    }

    m_odometry = new SwerveDriveOdometry(
        m_kinematics,
        getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
    
    // Update NetworkTables telemetry for all modules
    m_frontLeft.updateTelemetry();
    m_frontRight.updateTelemetry();
    m_backLeft.updateTelemetry();
    m_backRight.updateTelemetry();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    rot *= DriveConstants.kMaxAngularSpeedRadiansPerSecond;

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    
    setModuleStates(swerveModuleStates);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backLeft.resetEncoders();
    m_backRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_pigeon.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_pigeon.getYaw().getValueAsDouble();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_pigeon.getRate();
  }

  /**
   * Returns the rotation of the robot as a Rotation2d.
   *
   * @return The rotation of the robot.
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  /**
   * Stops all modules.
   */
  public void stopModules() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  @Override
  public void simulationPeriodic() {
    // Update module simulations
    m_frontLeft.simulationPeriodic();
    m_frontRight.simulationPeriodic();
    m_backLeft.simulationPeriodic();
    m_backRight.simulationPeriodic();

    // Calculate the angular velocity based on module states
    // This is a simplified simulation - in a real physics sim, you'd integrate velocities
    if (RobotBase.isSimulation()) {
      SwerveModuleState[] states = new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState()
      };
      
      ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(states);
      
      if (m_pigeonSim != null) {
        final double dt = 0.02;
        m_simulatedYawRadians =
            MathUtil.angleModulus(m_simulatedYawRadians + chassisSpeeds.omegaRadiansPerSecond * dt);
        m_pigeonSim.setRawYaw(Math.toDegrees(m_simulatedYawRadians));
        m_pigeonSim.setAngularVelocityZ(Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond));
      }
    }
  }
}
