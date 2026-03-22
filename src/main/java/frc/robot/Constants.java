
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
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
    public static double kMaxSpeedMetersPerSecond = 10;//5
  }

  // ============================================================================
  // MODULE CONSTANTS
  // ============================================================================
  public static final class ModuleConstants {
    // ===== Module Speed & Acceleration Constraints =====
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 100 * Math.PI;//8
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 6 * Math.PI;//4

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
  // VISION CONSTANTS
  // ============================================================================
  public static final class VisionConstants {
    // hub constants
    public static int hubTargetID = 9;
    public static final double hubHeightMeters = 0.64135; // 25.25 inches
    // Turn PID control
    public static final double VISION_TURN_kP = .25; // similar to kMaxModuleAngularAccelerationRadiansPerSecondSquared
    public static final double VISION_TURN_kI = 0.000;
    public static final double VISION_TURN_kD = 0.012;
    public static final double VISION_TURN_OUTPUT_DEADBAND = 0.00;//seems more stable without deadband
    // Forward PID control
    public static final double VISION_FORWARD_kP = 15; // (m/s)
    public static final double VISION_FORWARD_kI = 0.0005;
    public static final double VISION_FORWARD_kD = 1;
    public static final double VISION_FORWARD_OUTPUT_DEADBAND = 0.005;
    //stafe PID control
    public static final double VISION_STRAFE_kP = 0.3; // (m/s)
    public static final double VISION_STRAFE_kI = 0.0000;
    public static final double VISION_STRAFE_kD = 0.01;
    public static final double VISION_STRAFE_OUTPUT_DEADBAND = 0.005;
    // Camera constants
    public static final double cameraHeightMeters = 0.1063625; // 4.1825 inches
    //public static final double cameraYawSpeed = 0.1 * Math.PI; // similar to kMaxModuleAngularAccelerationRadiansPerSecondSquared
    //public static final double cameraRangeSpeed = 0.5; // (m/s)
  }

  // ============================================================================
  // OPERATOR INTERFACE CONSTANTS
  // ============================================================================
  public static final class OIConstants {
    // ===== Controller Ports =====
    public static final int kDriverControllerPort = 0;
    // ===== Controller Constants =====
    public static int driveForwardAxis = 1;
    public static int driveStrafeAxis = 0;
    public static int driveTurnAxis = 4;
    // Auto Selected
    public static int autoSelected = 0;

    // Motor Ports
    public static final int intakeSwingerPotentiometerPort = 0; // PWM channel
    public static final int intakeSwingerPort = 9; // PWM channel
    public static final int intakeRunnerKrakenPort = 10;
    public static final int ballRollerNeoPort = 11;
    public static final int shooterMotorPort1 = 12;
    public static final int shooterMotorPort2 = 13;
    public static final int shooterMotorPort3 = 14;
    public static final int shooterMotorPort4 = 15;
  }

  public static final class PIDConstants {
    public static final class IntakeSwingerConstants { // This motor is a neo
      public static final double kP = 0.00014;
      public static final double kI = 0.00004;
      public static final double kD = 0.00009;
      public static final double potentiometerTargetDegrees = 100.0;
      public static final double potentiometerDegrees = 270.0;
      public static final double potentiometerOffset = 0.0;
      public static final double ArmDegrees = potentiometerDegrees / 2;//1:2 ratio
      //get degrees by multipling the value returned by the ADC with the Arm degrees constant
    }
    public static final class BallRollerConstants { // This motor is a neo
      public static final double kP = 0.00014;
      public static final double kI = 0.00004;
      public static final double kD = 0.00009;
    }
    public static final class ShooterConstants { // 4 neo motors
      public static final double shooterSpeedRPM = 3000;
      public static final double kP = 0.00014;
      public static final double kI = 0.00004;
      public static final double kD = 0.00009;
      public static double kFF_LinearResponce = 5654;//5654
      public static double kFeedForward = shooterSpeedRPM / kFF_LinearResponce;
    }
  }

  // ============================================================================
  // AUTO CONSTANTS
  // ============================================================================
  public static final class AutoConstants {
    // ===== Speed & Acceleration Limits =====
    public static final double kMaxAutoSpeedMetersPerSecond = 1.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
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
