// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.epilogue.Logged;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import frc.robot.Constants.OIConstants;

// @Logged
// public class ShooterSubsystem extends SubsystemBase {
//   public WPI_TalonSRX m_shooterMotor1 = new WPI_TalonSRX(OIConstants.shooterMotorPort1);
//   public WPI_TalonSRX m_shooterMotor2 = new WPI_TalonSRX(OIConstants.shooterMotorPort2); 
//   private double shooterMotorSpeed = OIConstants.shooterSpeed;
//   /** Creates a new ExampleSubsystem. */
//   public ShooterSubsystem() {}

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
//   public void shooterIn(){
//     m_shooterMotor1.set(shooterMotorSpeed);
//     m_shooterMotor2.set(-shooterMotorSpeed);
//   }
//   public void shooterOut(){
//     m_shooterMotor1.set(-shooterMotorSpeed);
//     m_shooterMotor2.set(shooterMotorSpeed);
//   }
//   public void stop(){
//     m_shooterMotor1.stopMotor();
//     m_shooterMotor2.stopMotor();
//   }
// }
