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

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants.PIDConstants;
import edu.wpi.first.wpilibj.motorcontrol.Talon;


@Logged
public class ShooterSubsystem extends SubsystemBase {
  public class NeoResponse  {
    public final double power;
    public final double neoRPM;

    public NeoResponse(double pw, double rpm){
      power = pw;
      neoRPM = rpm;
    }
  }
  // Analog Output for the potentiometer is from 0 - 5
  // Based on last test
  private static final AnalogInput m_potADC = new AnalogInput(OIConstants.intakeSwingerPotentiometerPort);
  private static final AnalogPotentiometer m_potentiometer = new AnalogPotentiometer(m_potADC, 270, 30);
  private static final Talon intakeSwinger = new Talon(OIConstants.intakeSwingerPort);
  private static final TalonSRX intakeRunnerKraken = new TalonSRX(OIConstants.intakeRunnerKrakenPort);
  private static final Talon ballRoller = new Talon(OIConstants.ballRollerPort);
  private static final SparkMax shooterNeo1 = new SparkMax(OIConstants.shooterMotorPort1, MotorType.kBrushless);
  private static final SparkFlex shooterNeo2 = new SparkFlex(OIConstants.shooterMotorPort2, MotorType.kBrushless);
  private static final SparkMax shooterNeo3 = new SparkMax(OIConstants.shooterMotorPort3, MotorType.kBrushless);
  private static final SparkFlex shooterNeo4 = new SparkFlex(OIConstants.shooterMotorPort4, MotorType.kBrushless);
  private static final Talon feederMotor = new Talon(OIConstants.FeederPort); 
  // static final SparkMax feederMotor = new SparkMax(OIConstants.feederMotorPort, MotorType.kBrushless);
  private static final double sumOfSwingerPIDS = 
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
  private static double kFeedForward;

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
      // Configure the shooter motors to follow each other
      NeoResponse[] neoLookupTable = 
      {
        new NeoResponse(0.05, 250),
        new NeoResponse(0.1, 580),
        new NeoResponse(0.15, 800),
        new NeoResponse(0.2, 1200),
        new NeoResponse(0.25, 1500),
        new NeoResponse(0.3, 1800),
        new NeoResponse(0.35, 2050),
        new NeoResponse(0.4, 2400),
        new NeoResponse(0.45, 2700),
        new NeoResponse(0.5, 3000),
        new NeoResponse(0.55, 3300),
        new NeoResponse(0.6, 3550),
        new NeoResponse(0.65, 3900),
        new NeoResponse(0.7, 4150),
        new NeoResponse(0.75, 4450),
        new NeoResponse(0.8, 4700),
        new NeoResponse(0.85, 5000),
        new NeoResponse(0.9, 5250),
        new NeoResponse(0.95, 5500),
        new NeoResponse(1.0, 5750),
      };
      int index = 0;
      for(int i = 0; i < 10; i++)
        index += (neoLookupTable[i].neoRPM >= PIDConstants.ShooterConstants.shooterSpeedRPM) ? 1: 0;   
      
      kFeedForward = neoLookupTable[index].power;
      //shooterNeo1.setInverted(true);
      //shooterNeo2.setInverted(true);
      //shooterNeo4.setInverted(true);

    }

  /**
   * Swing the intake out to pick up balls
   */
  public void swingOut() {
    // Swing the intake out
    double targetPosition = PIDConstants.IntakeSwingerConstants.potentiometerTargetDegrees;
    double currentPosition = (m_potADC.getValue() / PIDConstants.IntakeSwingerConstants.potMaxValue) * PIDConstants.IntakeSwingerConstants.ArmDegrees; // Current Arm position in degrees
    double output = intakeSwingerPID.calculate(currentPosition, targetPosition);
    intakeSwinger.set(0);

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
    intakeSwinger.set(0);

    // Stop intaking
    intakeRunnerKraken.set(TalonSRXControlMode.PercentOutput, 0);
  }

  /**
   * Roll the balls into the shooter
   */
  public void rollBalls() {
    ballRoller.set(0.1);
  }

  /**
   * Stop the ball roller
   */
  public void stopRoller() {
    ballRoller.set(0);
  }

  /**
   * Shoot
   */
  public void shoot() {
    double targetRPM = PIDConstants.ShooterConstants.shooterSpeedRPM;
    double currentRPM = shooterNeo1.getEncoder().getVelocity();
    double output = shooterPID.calculate(currentRPM, targetRPM) + kFeedForward;
    //if(currentRPM >= PIDConstants.ShooterConstants.feedingThreshholdRPM){
      //feederMotor.set(PIDConstants.ShooterConstants.feederMotorSpeed);
    //}

    shooterNeo1.set(-output);
    shooterNeo2.set(-output);
    shooterNeo3.set(-output);
    shooterNeo4.set(output); 
  }

  /**
   * Stop shooting
   */
  public void stopShooter(){
    double targetRPM = 0;
    double currentRPM = shooterNeo1.getEncoder().getVelocity();
    double output = shooterPID.calculate(currentRPM, targetRPM);
    shooterNeo1.set(0);
    shooterNeo2.set(0);
    shooterNeo3.set(0);
    shooterNeo4.set(0);  
    feederMotor.set(0);
  }

  /**
   * 
   * @param kP
   * @param kI
   * @param kD
   * 
   * Creates a new PID controller for the intake swinger if the
   *  given values are different from the values already there
   */
  public void updateSwingerPID(double kP, double kI, double kD) {
    if(!(kP*kI*kD == productOfSwingerPIDS && kP+kI+kD == sumOfSwingerPIDS)){
      intakeSwingerPID = new PIDController(kP, kI, kD);
    }
  }

  public AnalogInput getPotADC(){
    return m_potADC;
  }

  public AnalogPotentiometer getPotentiometer(){
    return m_potentiometer;
  }

  /**
   * Emergency method to stop all non-drive motors
   * Stop all motors in the shooter subsystem
   */
  public void stopAllMotors() {
    intakeSwinger.stopMotor();
    intakeRunnerKraken.set(TalonSRXControlMode.PercentOutput, 0);
    ballRoller.stopMotor();
    shooterNeo1.stopMotor();
    shooterNeo2.stopMotor();
    shooterNeo3.stopMotor();
    shooterNeo4.stopMotor();
    feederMotor.stopMotor();
  }
}
