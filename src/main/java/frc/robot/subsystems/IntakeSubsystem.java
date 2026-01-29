// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.epilogue.Logged;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.ctre.phoenix6.motorcontrol.can.WPI_TalonSRX;
// import frc.robot.Constants.OIConstants;

// @Logged
// public class IntakeSubsystem extends SubsystemBase {
//   public WPI_TalonSRX m_intakeMotor1 = new WPI_TalonSRX(OIConstants.intakeMotorPort1);
//   public WPI_TalonSRX m_intakeMotor2 = new WPI_TalonSRX(OIConstants.intakeMotorPort2); 
//   private double intakeMotorSpeed = OIConstants.intakeSpeed;
//   /** Creates a new ExampleSubsystem. */
//   public IntakeSubsystem() {}

//   /**
//    * Example command factory method.
//    *
//    * @return a command
//    */
//   public Command exampleMethodCommand() {
//     // Inline construction of command goes here.
//     // Subsystem::RunOnce implicitly requires `this` subsystem.
//     return runOnce(
//         () -> {
//           /* one-time action goes here */
//         });
//   }
//   /**
//    * An example method querying a boolean state of the subsystem (for example, a digital sensor).
//    *
//    * @return value of some boolean subsystem state, such as a digital sensor.
//    */
//   public void intakeIn(){
//     m_intakeMotor1.set(intakeMotorSpeed);
//     m_intakeMotor2.set(-intakeMotorSpeed);
//   }
//   public void intakeOut(){
//     m_intakeMotor1.set(-intakeMotorSpeed);
//     m_intakeMotor2.set(intakeMotorSpeed);
//   }
//   public void stop(){
//     m_intakeMotor1.stopMotor();
//     m_intakeMotor2.stopMotor();
//   }
// }
