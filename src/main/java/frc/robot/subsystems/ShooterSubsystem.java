// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.PIDConstants;
import frc.robot.RobotContainer;
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
  private static XboxController m_driverController;
  private static final AnalogInput m_potADC = new AnalogInput(OIConstants.intakeSwingerPotentiometerPort);
  private static final AnalogPotentiometer m_potentiometer = new AnalogPotentiometer(m_potADC, 270, 30);
  private static final Talon intakeSwinger = new Talon(OIConstants.intakeSwingerPort);
  private static final TalonFX intakeRunnerKraken = new TalonFX(OIConstants.intakeRunnerKrakenPort);
  private static final Talon ballRoller = new Talon(OIConstants.ballRollerPort);
  private static final SparkMax shooterNeo1 = new SparkMax(OIConstants.shooterMotorPort1, MotorType.kBrushless);
  private static final SparkFlex shooterNeo2 = new SparkFlex(OIConstants.shooterMotorPort2, MotorType.kBrushless);
  private static final SparkMax shooterNeo3 = new SparkMax(OIConstants.shooterMotorPort3, MotorType.kBrushless);
  private static final SparkFlex shooterNeo4 = new SparkFlex(OIConstants.shooterMotorPort4, MotorType.kBrushless);
  private static final Talon feederMotor = new Talon(OIConstants.FeederPort); 
  private static final DigitalInput topArmLimitSwitch = new DigitalInput(0);
  private static final DigitalInput bottomArmLimitSwitch = new DigitalInput(1);
  private static boolean shooterOn = false;
  private static boolean krakenOn = false;
  private static double intakeKrakenSpeed = -1;
  private static double ballRollerSpeed = -0.5;
    
  private static final double sumOfShooterPIDS = 
  PIDConstants.ShooterConstants.kP 
  + PIDConstants.ShooterConstants.kI
  + PIDConstants.ShooterConstants.kD;
  private final double productOfShooterPIDS = 
  PIDConstants.ShooterConstants.kP 
  * PIDConstants.ShooterConstants.kI
  * PIDConstants.ShooterConstants.kD;
  // PID Controller for the shooters
  private static PIDController shooterPID = new PIDController(
  PIDConstants.ShooterConstants.kP,
  PIDConstants.ShooterConstants.kI,
  PIDConstants.ShooterConstants.kD
  );
  private static double kFeedForward;
  
  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem(XboxController driveController) {
    m_driverController = driveController;
    calculateFeedForwardForShooter();
    // Configure the shooter motors to follow each other
    
    //shooterNeo1.setInverted(true);
    //shooterNeo2.setInverted(true);
    //shooterNeo4.setInverted(true);
    
  }

  public void calculateFeedForwardForShooter(){
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
    for(int i = 0; i < neoLookupTable.length; i++)
    index += (neoLookupTable[i].neoRPM <= PIDConstants.ShooterConstants.shooterSpeedRPM) ? 1: 0;   
    
    kFeedForward = neoLookupTable[index].power;
  }

  /**
   * positive set speed resulted in swinger comming in
   * Button A is for swing in
   * Button B is for swing out
   */
  public void swingerHandler(){
    // double currentPosition = (m_potADC.getValue() / PIDConstants.IntakeSwingerConstants.potMaxValue) * PIDConstants.IntakeSwingerConstants.potentiometerDegrees; // Current Arm position in degrees
    if(m_driverController.getAButton()) // swing in if button A is held
    {
      if(topArmLimitSwitch.get()) 
        intakeSwinger.set(PIDConstants.IntakeSwingerConstants.swingerSpeed); // Since the default value of the limit switch is true when not pressed
      else 
        intakeSwinger.set(0); // Limit switch is pressed, so stop moving
    }
    else if (m_driverController.getBButton()) // swing out if button B is held
    {
      if(bottomArmLimitSwitch.get()) // Since the default value of the limit switch is true when not pressed
        intakeSwinger.set(-PIDConstants.IntakeSwingerConstants.swingerSpeed); 
      else 
        intakeSwinger.set(0); // Limist switch is pressed, so stop moving
    }
    else
    {
      intakeSwinger.stopMotor();
    }
  }

  public void shooterHandler(){
    if(m_driverController.getLeftBumperButtonPressed())
      shooterOn = !shooterOn;

    if(m_driverController.getLeftTriggerAxis()>0.5) PIDConstants.ShooterConstants.shooterSpeedRPM = 2200;
    else PIDConstants.ShooterConstants.shooterSpeedRPM = 2900;

    if(shooterOn)
      shoot();
    else
      stopShooter();
  }

  public void krakenHandler(){
    if(m_driverController.getRightBumperButtonPressed())
      krakenOn = !krakenOn;
    
    if(m_driverController.getXButton())
    {
      intakeKrakenSpeed *= -1;
      ballRollerSpeed *= -1;
    }

    if(krakenOn)
      enableIntakeKraken();
    else
      disableIntakeKraken();
  }

  public boolean isKrakenOn(){return krakenOn;}
  public boolean isShooterOn(){return shooterOn;}

  public boolean getTopLimitSwitch(){
    return topArmLimitSwitch.get();
  }

  public boolean getBottomLimitSwitch(){
    return bottomArmLimitSwitch.get();
  }

  public double getCalculatedCurrentPosition(){
    return (m_potADC.getValue() / PIDConstants.IntakeSwingerConstants.potMaxValue) * PIDConstants.IntakeSwingerConstants.potentiometerDegrees; // Current Arm position in degrees
  }

  public void enableIntakeKraken(){
    intakeRunnerKraken.set(intakeKrakenSpeed);
    rollBalls();
  }

  public void disableIntakeKraken(){
    intakeRunnerKraken.set(0);
    stopRoller();
  }
  
  /**
  * Roll the balls into the shooter
  */
  public void rollBalls() {
    ballRoller.set(ballRollerSpeed);
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
    calculateFeedForwardForShooter();
    double targetRPM = PIDConstants.ShooterConstants.shooterSpeedRPM;
    double currentRPM = Math.abs(shooterNeo1.getEncoder().getVelocity());
    double feedingThreshhold = PIDConstants.ShooterConstants.feedingThreshholdRPM;
    double output = shooterPID.calculate(currentRPM, targetRPM) + kFeedForward;
    
    if(currentRPM > targetRPM - PIDConstants.ShooterConstants.IresetThreshold) shooterPID.reset();
    
    if(currentRPM < targetRPM - feedingThreshhold)
    {
      feederMotor.set(0);
    }
    else
    {
      feederMotor.set(PIDConstants.ShooterConstants.feederMotorSpeed);
    }
    shooterNeo1.set(-output);
    shooterNeo2.set(-output);
    shooterNeo3.set(-output);
    shooterNeo4.set(output);
  }
  
  /**
  * Stop shooting
  */
  public void stopShooter(){
    shooterPID.reset();
    shooterNeo1.set(0);
    shooterNeo2.set(0);
    shooterNeo3.set(0);
    shooterNeo4.set(0); 
    feederMotor.set(0);
  }

  public double getShooterRPM(){
    return shooterNeo1.getEncoder().getVelocity();
  }

  /**
  * 
  * @param kP
  * @param kI
  * @param kD
  * 
  * Creates a new PID controller for the shooter if the
  *  given values are different from the values already there
  */
  public void updateShooterPID(double kP, double kI, double kD) {
    if(kP*kI*kD != productOfShooterPIDS || kP+kI+kD != sumOfShooterPIDS){
      shooterPID = new PIDController(kP, kI, kD);
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
    intakeRunnerKraken.set(0);
    ballRoller.stopMotor();
    shooterNeo1.stopMotor();
    shooterNeo2.stopMotor();
    shooterNeo3.stopMotor();
    shooterNeo4.stopMotor();
    feederMotor.stopMotor();
  }
}
