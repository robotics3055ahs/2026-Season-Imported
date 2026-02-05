
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // ============================================================================
  // DRIVE CONSTANTS
  // ============================================================================
  public static final class DriveConstants {
    // ===== Feature Flags =====
    public static final boolean enableDrive = true;
    public static final boolean enableVision = true;

    // ===== Drive Motor Ports =====
    public static final int kFrontLeftDriveMotorPort = enableDrive ? 3 : 0;
    public static final int kRearLeftDriveMotorPort = enableDrive ? 5 : 0;
    public static final int kFrontRightDriveMotorPort = enableDrive ? 7 : 0;
    public static final int kRearRightDriveMotorPort = enableDrive ? 1 : 0;

    // ===== Turning Motor Ports =====
    public static final int kFrontLeftTurningMotorPort = enableDrive ? 4 : 0;
    public static final int kRearLeftTurningMotorPort = enableDrive ? 6 : 0;
    public static final int kFrontRightTurningMotorPort = enableDrive ? 8 : 0;
    public static final int kRearRightTurningMotorPort = enableDrive ? 2 : 0;

    // ===== Turning Encoder Ports =====
    public static final int kFrontLeftTurningEncoderPorts = 2;
    public static final int kRearLeftTurningEncoderPorts = 3;
    public static final int kFrontRightTurningEncoderPorts = 4;
    public static final int kRearRightTurningEncoderPorts = 1;

    // ===== Turning Encoder Reversed Configuration =====
    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kRearRightTurningEncoderReversed = true;

    // ===== Drive Encoder Reversed Configuration =====
    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final boolean kRearRightDriveEncoderReversed = true;

    // ===== Drive Period =====
    // If you call DriveSubsystem.drive() with a different period make sure to update this.
    public static final double kDrivePeriod = 0.02;

    // ===== Drivetrain Dimensions =====
    public static final double kTrackWidth = 0.63;  // Distance between centers of right and left wheels
    public static final double kWheelBase = 0.58;   // Distance between front and back wheels

    // ===== Kinematics =====
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // ===== Gyro Configuration =====
    public static final boolean kGyroReversed = false;

    // ===== Drive Characterization Constants =====
    public static final double ksVolts = 1.0;
    public static final double kvVoltSecondsPerMeter = 0.12;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;
    public static final double kVoltsPerRotation = 60 / 523.0;
    public static final double kVoltPerMeterPerSecond = 0.413;

    // ===== Speed Limits =====
    public static final double kMaxSpeedMetersPerSecond = 0.1;
  }

  // ============================================================================
  // MODULE CONSTANTS
  // ============================================================================
  public static final class ModuleConstants {
    // ===== Module Speed & Acceleration Constraints =====
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 4 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 4 * Math.PI;

    // ===== Encoder Configuration =====
    public static final int kEncoderCPR = 4096;
    public static final double kWheelDiameterMeters = 0.102;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
    public static final double kDriveEncoderDistancePerRotation =
        (kWheelDiameterMeters * Math.PI) / 6.75;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kEncoderCPR;

    // ===== Drive Motor PID Controller =====
    public static final double kPModuleDriveController = 0.7;
    public static final double kIModuleDriveController = 0.0;
    public static final double kDModuleDriveController = 0.001;

    // ===== Drive Motor Feedforward (Static & Velocity) =====
    public static final double ksModuleDriveController = 0.0;
    public static final double kvModuleDriveController = 3.0;

    // ===== Turning Motor PID Controller =====
    public static final double kPModuleTurningController = 10.0;
    public static final double kIModuleTurningController = 0.0;
    public static final double kDModuleTurningController = 0.0;

    // ===== Turning Motor Feedforward (Static & Velocity) =====
    public static final double ksModuleTurningController = 0.0;
    public static final double kvModuleTurningController = 0.0;
  }

  // ============================================================================
  // LADDER CONSTANTS
  // ============================================================================
  public static final class LadderConstants {
    // ===== Motor Ports =====
    public static final int ladderMotorPort1 = 21;
    public static final int ladderMotorPort2 = 22;

    // ===== Motor Speed =====
    public static final double ladderMotorSpeed = 1.00;

    // ===== Position Setpoints =====
    public static final double topStalkPosition = 26.5;
    public static final double middleStalkPosition = 14.14;
    public static final double bottomStalkPosition = 6.07;
    public static final double zeroPosition = 0.0;
  }

  // ============================================================================
  // VISION CONSTANTS
  // ============================================================================
  public static final class VisionConstants {
    public static final int hubTargetID = 22;
  }

  // ============================================================================
  // OPERATOR INTERFACE CONSTANTS
  // ============================================================================
  public static final class OIConstants {
    // ===== Controller Ports =====
    public static final int kDriverControllerPort = 0;

    // ===== Shooter Motor Ports =====
    public static final int shooterMotorPort1 = 30;
    public static final int shooterMotorPort2 = 31;

    // ===== Shooter Speed =====
    public static final double shooterSpeed = 0.75;

    // ===== Intake Motor Ports =====
    public static final int intakeMotorPort1 = 40;
    public static final int intakeMotorPort2 = 41;

    // ===== Intake Speed =====
    public static final double intakeSpeed = 0.75;
  }

  // ============================================================================
  // AUTO CONSTANTS
  // ============================================================================
  public static final class AutoConstants {
    // ===== Speed & Acceleration Limits =====
    public static final double kMaxAutoSpeedMetersPerSecond = 1.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    // ===== Position & Rotation PID Controllers =====
    public static final double kPXController = 1.0;
    public static final double kPYController = 1.0;
    public static final double kPThetaController = 1.0;

    // ===== Motion Profile Constraints =====
    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
