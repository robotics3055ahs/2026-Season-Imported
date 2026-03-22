// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants.PIDConstants;
import edu.wpi.first.wpilibj.motorcontrol.Talon;


@Logged
public class ShooterSubsystem extends SubsystemBase {
  private final AnalogInput m_potADC = new AnalogInput(OIConstants.intakeSwingerPotentiometerPort);
  private final Talon intakeSwinger = new Talon(OIConstants.intakeSwingerPort);
  private final TalonSRX intakeRunnerKraken = new TalonSRX(OIConstants.intakeRunnerKrakenPort);
  private final SparkFlex ballRollerNeo = new SparkFlex(OIConstants.ballRollerNeoPort, MotorType.kBrushless);
  private final SparkMax shooterNeo1 = new SparkMax(OIConstants.shooterMotorPort1, MotorType.kBrushless);
  private final SparkMax shooterNeo2 = new SparkMax(OIConstants.shooterMotorPort2, MotorType.kBrushless);
  private final SparkMax shooterNeo3 = new SparkMax(OIConstants.shooterMotorPort3, MotorType.kBrushless);
  private final SparkMax shooterNeo4 = new SparkMax(OIConstants.shooterMotorPort4, MotorType.kBrushless);
  private final double sumOfSwingerPIDS = 
    PIDConstants.IntakeSwingerConstants.kP 
    + PIDConstants.IntakeSwingerConstants.kI
    + PIDConstants.IntakeSwingerConstants.kD;
  private final double productOfSwingerPIDS = 
    PIDConstants.IntakeSwingerConstants.kP 
    * PIDConstants.IntakeSwingerConstants.kI
    * PIDConstants.IntakeSwingerConstants.kD;

  // PID Controller for the intake swinger
  private static PIDController intakeSwingerPID = new PIDController(
    PIDConstants.IntakeSwingerConstants.kP,
    PIDConstants.IntakeSwingerConstants.kI,
    PIDConstants.IntakeSwingerConstants.kD
  );
  // PID Controller for the shooters
  private static PIDController shooterPID = new PIDController(
    PIDConstants.ShooterConstants.kP,
    PIDConstants.ShooterConstants.kI,
    PIDConstants.ShooterConstants.kD
  );

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
      // Configure the shooter motors to follow each other
      shooterNeo3.setInverted(true);
      shooterNeo4.setInverted(true);
  }

  /**
   * Swing the intake out to pick up balls
   */
  public void swingOut() {
    // Swing the intake out
    double targetPosition = PIDConstants.IntakeSwingerConstants.potentiometerTargetDegrees;
    double currentPosition = m_potADC.getValue() * PIDConstants.IntakeSwingerConstants.ArmDegrees; // Current Arm position in degrees
    double output = intakeSwingerPID.calculate(currentPosition, targetPosition);
    intakeSwinger.set(output);

    // Run the swinger arm motors to pick up balls
    intakeRunnerKraken.set(TalonSRXControlMode.PercentOutput, 0.1);
  }

  /**
   * Swing the intake back in 
   */
  public void swingIn() {
    double targetPosition = 0;
    double currentPosition = m_potADC.getValue() * PIDConstants.IntakeSwingerConstants.ArmDegrees; // Current Arm position in degrees
    double output = intakeSwingerPID.calculate(currentPosition, targetPosition);
    intakeSwinger.set(output);

    // Stop intaking
    intakeRunnerKraken.set(TalonSRXControlMode.PercentOutput, 0);
  }

  /**
   * Roll the balls into the shooter
   */
  public void rollBalls() {
    ballRollerNeo.set(0.1);
  }

  /**
   * Stop the ball roller
   */
  public void stopRoller() {
    ballRollerNeo.set(0);
  }

  /*
   * Shoot
   */
  public void shoot() {
    double targetRPM = PIDConstants.ShooterConstants.shooterSpeedRPM;
    double currentRPM1 = shooterNeo1.getEncoder().getVelocity() + PIDConstants.ShooterConstants.kFeedForward;
    double currentRPM2 = shooterNeo2.getEncoder().getVelocity() + PIDConstants.ShooterConstants.kFeedForward;
    double currentRPM3 = shooterNeo3.getEncoder().getVelocity() + PIDConstants.ShooterConstants.kFeedForward;
    double currentRPM4 = shooterNeo4.getEncoder().getVelocity() + PIDConstants.ShooterConstants.kFeedForward;
    
    double output1 = shooterPID.calculate(currentRPM1, targetRPM);
    double output2 = shooterPID.calculate(currentRPM2, targetRPM);
    double output3 = shooterPID.calculate(currentRPM3, targetRPM);
    double output4 = shooterPID.calculate(currentRPM4, targetRPM);

    shooterNeo1.set(output1);
    shooterNeo2.set(output2);
    shooterNeo3.set(output3);
    shooterNeo4.set(output4);  
  }

  /*
   * Stop shooting
   */
  public void stopShooter(){
    double targetRPM = 0;
    double currentRPM1 = shooterNeo1.getEncoder().getVelocity() + PIDConstants.ShooterConstants.kFeedForward;
    double currentRPM2 = shooterNeo2.getEncoder().getVelocity() + PIDConstants.ShooterConstants.kFeedForward;
    double currentRPM3 = shooterNeo3.getEncoder().getVelocity() + PIDConstants.ShooterConstants.kFeedForward;
    double currentRPM4 = shooterNeo4.getEncoder().getVelocity() + PIDConstants.ShooterConstants.kFeedForward;
    
    double output1 = shooterPID.calculate(currentRPM1, targetRPM);
    double output2 = shooterPID.calculate(currentRPM2, targetRPM);
    double output3 = shooterPID.calculate(currentRPM3, targetRPM);
    double output4 = shooterPID.calculate(currentRPM4, targetRPM);

    shooterNeo1.set(output1);
    shooterNeo2.set(output2);
    shooterNeo3.set(output3);
    shooterNeo4.set(output4);  
  }

  public void updateSwingerPID(double kP, double kI, double kD) {
    if(!(kP*kI*kD == productOfSwingerPIDS && kP+kI+kD == sumOfSwingerPIDS)){
      intakeSwingerPID = new PIDController(kP, kI, kD);
    }
  }

  /**
   * Emergency method to stop all non-drive motors
   * Stop all motors in the shooter subsystem
   */
  public void stopAllMotors() {
    intakeSwinger.stopMotor();
    intakeRunnerKraken.set(TalonSRXControlMode.PercentOutput, 0);
    ballRollerNeo.stopMotor();
    shooterNeo1.stopMotor();
    shooterNeo2.stopMotor();
    shooterNeo3.stopMotor();
    shooterNeo4.stopMotor();
  }
}
