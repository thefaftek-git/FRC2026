// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kKeyboardPort = 1;
    
    // Controller deadband
    public static final double kDeadband = 0.1;
    
    // Keyboard keys (USB HID keycodes)
    public static final int kKeyW = 26; // W key
    public static final int kKeyA = 4;  // A key
    public static final int kKeyS = 22; // S key
    public static final int kKeyD = 7;  // D key
    public static final int kKeyQ = 20; // Q key
    public static final int kKeyE = 8;  // E key
    
    // Keyboard speed multiplier (slower than joystick for precision)
    public static final double kKeyboardSpeed = 0.6;
  }

  public static class DriveConstants {
    // Swerve drive configuration for 28x28 inch drivebase
    public static final double kTrackWidth = 0.7112; // 28 inches in meters
    public static final double kWheelBase = 0.7112; // 28 inches in meters
    
    // MK4n module constants with 7.13:1 gear ratio
    public static final double kDriveGearRatio = 7.13;
    public static final double kWheelDiameterMeters = 0.1016; // 4 inches in meters
    public static final double kDriveMotorRotationsPerMeter = 
        kDriveGearRatio / (kWheelDiameterMeters * Math.PI);
    
    // Maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.5; // Adjust based on testing
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2; // 2 rotations per second
    
    // CAN IDs - Update these to match your robot configuration
    // Front Left Module
    public static final int kFrontLeftDriveMotorId = 1;
    public static final int kFrontLeftTurningMotorId = 2;
    public static final int kFrontLeftCanCoderId = 9;
    
    // Front Right Module
    public static final int kFrontRightDriveMotorId = 3;
    public static final int kFrontRightTurningMotorId = 4;
    public static final int kFrontRightCanCoderId = 10;
    
    // Back Left Module
    public static final int kBackLeftDriveMotorId = 5;
    public static final int kBackLeftTurningMotorId = 6;
    public static final int kBackLeftCanCoderId = 11;
    
    // Back Right Module
    public static final int kBackRightDriveMotorId = 7;
    public static final int kBackRightTurningMotorId = 8;
    public static final int kBackRightCanCoderId = 12;
    
    // Pigeon 2.0 CAN ID
    public static final int kPigeonId = 13;
    
    // Turning motor PID constants - Tune these values
    public static final double kTurningP = 0.5;
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.0;
    
    // Drive motor PID constants - Tune these values
    public static final double kDriveP = 0.1;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;
    
    // Drive idle deadband to prevent tiny corrections when stopped
    public static final double kDriveIdleDeadbandMetersPerSecond = 0.02;

    // Encoder offsets in radians - Calibrate these for your robot
    public static final double kFrontLeftEncoderOffset = 0.0;
    public static final double kFrontRightEncoderOffset = 0.0;
    public static final double kBackLeftEncoderOffset = 0.0;
    public static final double kBackRightEncoderOffset = 0.0;

    // Simulation tuning constants
    public static final double kSimSupplyVoltage = 12.0;
    public static final double kDriveSimVelocityResponse = 6.0; // Larger -> faster response
    public static final double kTurningSimVelocityResponse = 10.0;
    public static final double kDriveSimVelocityDeadband = 1e-3;
    public static final double kTurningSimVelocityDeadband = 1e-3;
  }
}
