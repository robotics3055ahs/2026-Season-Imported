// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.DriveConstants;

@Logged
public class SwerveModule {
  public final TalonFX m_driveMotor;
  public final TalonFX m_turningMotor;
  private final CANcoder m_turningEncoderNew;
  
  private final int turningEncoderReversed;
  private final int driveEncoderReversed;

  public SwerveModulePosition swervePosition;
  public double moduleDistance;
  public CommandJoystick stick = new CommandJoystick(3);

  public int driveChannel;

  private final ProfiledPIDController m_drivePIDController =
      new ProfiledPIDController(
        ModuleConstants.kPModuleDriveController,
        0,//.0020,
        0.001,
        new TrapezoidProfile.Constraints(
          DriveConstants.kMaxSpeedMetersPerSecond,
          1000));

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0, 3);


  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param driveEncoderChannels The channels of the drive encoder.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
        int turningEncoderChannels,
      Boolean driveEncoderReversedBool,
      Boolean turningEncoderReversedBool) {

    driveChannel = driveMotorChannel;

    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);
    m_turningEncoderNew = new CANcoder(turningEncoderChannels);
    
    turningEncoderReversed = (turningEncoderReversedBool == true) ? -1 : 1;
    driveEncoderReversed = (driveEncoderReversedBool == true) ? -1 : 1;
    
    //m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
    //m_turningMotor.setNeutralMode(NeutralModeValue.Brake);
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   * 
   */

  public SwerveModuleState getState() {
  
    return new SwerveModuleState(
        //m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getDistance()));
        (-driveEncoderReversed * m_driveMotor.getVelocity().getValueAsDouble() * ModuleConstants.kDriveEncoderDistancePerRotation), new Rotation2d(m_turningEncoderNew.getPosition().getValue()));//.getValueAsDouble() * 2 * Math.PI));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    swervePosition = new SwerveModulePosition(
        //m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getDistance()));
        (driveEncoderReversed * m_driveMotor.getPosition().getValueAsDouble() * ModuleConstants.kDriveEncoderDistancePerRotation), new Rotation2d(m_turningEncoderNew.getPosition().getValue()));
    return swervePosition;
  }

  public double getTurnAngle(){
    //return m_turningEncoder.getDistance();
    return m_turningEncoderNew.getPosition().getValueAsDouble();
  }
  public double getSpeed(){
    return m_driveMotor.getVelocity().getValueAsDouble() * ModuleConstants.kDriveEncoderDistancePerRotation;
  }
  public double getModuleDistance(){
    //return m_driveEncoder.getDistance();
    moduleDistance = m_driveMotor.getPosition().getValueAsDouble() * ModuleConstants.kDriveEncoderDistancePerRotation;
    return moduleDistance;
  }
  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(m_turningEncoderNew.getPosition().getValue());
    desiredState.speedMetersPerSecond *= driveEncoderReversed;
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(encoderRotation);
    
    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    desiredState.cosineScale(encoderRotation);
    
    // Calculate the drive output from the drive PID controller.
    
    final double driveOutput = m_drivePIDController.calculate(m_driveMotor.getVelocity().getValueAsDouble() * ModuleConstants.kDriveEncoderDistancePerRotation, desiredState.speedMetersPerSecond);
    
    final double driveFeedforward = m_driveFeedforward.calculate(driveOutput);//desiredState.speedMetersPerSecond);
    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = //desiredState.angle.getRotations() - encoderRotation.getRotations();
      m_turningPIDController.calculate(m_turningEncoderNew.getAbsolutePosition().getValueAsDouble() * 2* Math.PI, desiredState.angle.getRadians());
    
    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.setVoltage(/*(driveOutput + */driveFeedforward);//desiredState.speedMetersPerSecond);
    m_turningMotor.setVoltage(turningEncoderReversed * turnOutput);
    
  }
  public void changeMaxSpeed(double Speed){
    m_drivePIDController.setConstraints(new TrapezoidProfile.Constraints(
      Speed,
      2));
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveMotor.setPosition(0);
    m_turningEncoderNew.setPosition(0);
  }

}
