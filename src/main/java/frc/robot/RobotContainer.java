// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.driveCommands.MoveToPosition;
import frc.robot.commands.PathMaker;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.fasterxml.jackson.annotation.JsonIgnore;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  //private final PowerDistribution m_PDP = new PowerDistribution();
  private final PathMaker m_PathMaker = new PathMaker();
  private final VisionSubsystem m_robotVision = new VisionSubsystem();
  private final PhotonCamera m_Camera = m_robotVision.getPhotonCamera();
  private boolean m_fieldRelative = true;
  public ShuffleboardTab tab;
  // Target varialbe for vision subsystem
  Pose2d poseToTarget;
  Transform2d transformToTarget;
  List<Translation2d> detectedTranslations2d;

  // The driver's controller
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  // private final Joystick m_driverRJoystick = new Joystick(OIConstants.kRightJoystickPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    
    configureButtonBindings();
    initDashboard();
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    // Multiply by max speed to map the joystick unitless inputs to actual units.
                    // This will map the [-1, 1] to [max speed backwards, max speed forwards],
                    // converting them to actual units.
                    Math.abs(m_driverController.getRawAxis(1)) > 0.05 ? -(m_driverController.getRawAxis(1)) * DriveConstants.kMaxSpeedMetersPerSecond : 0,
                    Math.abs(m_driverController.getRawAxis(0)) > 0.05 ? -(m_driverController.getRawAxis(0)) * DriveConstants.kMaxSpeedMetersPerSecond : 0,
                    Math.abs(m_driverController.getRawAxis(2)) > 0.3 ? -m_driverController.getRawAxis(2) * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond: 0,                    
                    m_fieldRelative),
            m_robotDrive));
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {  
    //new JoystickButton(m_driverController, 5).onTrue(new InstantCommand(() -> m_robotDrive.resetGyro(), m_robotDrive));
    new JoystickButton(m_driverController, 6).onTrue(new InstantCommand(() -> m_fieldRelative = !m_fieldRelative));
	  /* new JoystickButton(m_driverController, 5).whileTrue(
        new InstantCommand(() -> robotMoveToAprilTag())
    ); */
	
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_driverController.getRawButton(0)).onTrue();

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b(0).whileTrue();
  }

  public void initDashboard(){
    SmartDashboard.putNumber("Auto Selector", 0);
  }

  public void updateDashboard(){
    if(m_Camera.isConnected()){
      SmartDashboard.putBoolean("Camera Connected", true);
      PhotonTrackedTarget m_cameraTarget = m_robotVision.getCameraTarget();
      if(m_cameraTarget != null){
        SmartDashboard.putNumber("Camera Target ID", m_cameraTarget.getFiducialId());
        SmartDashboard.putNumber("Camera Target Pitch", m_cameraTarget.getPitch());
        SmartDashboard.putNumber("Camera Target Yaw", m_cameraTarget.getYaw());
        SmartDashboard.putNumber("Camera Target Area", m_cameraTarget.getArea());
        SmartDashboard.putString("Camera Target", m_cameraTarget.toString());
      } else {
        SmartDashboard.putString("Camera Target", "No Target");
      }
    } else {
      SmartDashboard.putBoolean("Camera Connected", false);
    }
    SmartDashboard.putBoolean("Field Relative?", m_fieldRelative);
    //SmartDashboard.putData("PDP Data", m_PDP);
    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
    SmartDashboard.putData("Drive Subsystem", m_robotDrive);
  }

  public void periodic() {
    updateDashboard();
    m_robotVision.updateCamera();
    //m_robotDrive.changeMaxSpeed(m_driverRJoystick.getRawAxis(0));    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(int autoNumber) {
    return new RunCommand(()->m_robotDrive.drive(-1,0,0, false), m_robotDrive).withTimeout(3)
		.andThen(()->System.out.println("Auto Step 1 Complete")).withTimeout(2)
		.andThen(()->m_robotDrive.drive(0,-1,0,false), m_robotDrive).withTimeout(3)
		.andThen(()->System.out.println("Auto Step 2 Complete")).withTimeout(2)
		.andThen(()->m_robotDrive.drive(0,0,1, false), m_robotDrive).withTimeout(3)
		.andThen(()->System.out.println("Auto Step 3 Complete")).withTimeout(2)
		.withTimeout(3);
   // return m_PathMaker.createPath(m_robotDrive, new Pose2d(1,0,new Rotation2d()), List.of(), null)
    //.andThen(new ReefMoveToPosition(2, 1, m_ladder, m_robotDrive, m_intake))
    //.andThen(new IntakeOut(m_intake));
    
  }
    //return (new MoveToPosition(m_robotDrive, new Pose2d(0,1,new Rotation2d()), List.of(), false).andThen(new IntakeIn(m_intake).withTimeout(2)))
    //  .andThen(new MoveToPosition(m_robotDrive, new Pose2d(1,0,new Rotation2d()), List.of(), false));


  //   if (Timer.getMatchTime() > 13) {
  //   m_robotDrive.m_frontLeft.m_driveMotor.set(0.4);
  //   m_robotDrive.m_frontLeft.m_turningMotor.set(autoNumber);
  // }   
  /*return pathMaker.createPath(
    m_robotDrive,
    new Pose2d(1, 0, new Rotation2d(0)),
    List.of(),
    false);*/
    //.andThen(pathMaker.createPath(
    //m_robotDrive,
    //new Pose2d(0,0.5, new Rotation2d(0)),
    //List.of(),
    //false));
  
  //ParallelRaceGroup cmd = new RunCommand(()->m_robotDrive.drive(2,0,0,false)).withTimeout(3);
  // return cmd;
    
      
      /*.andThen(pathMaker.createPath(
      m_robotDrive,
      new Pose2d(-3,-3, new Rotation2d(0)),
      List.of(new Translation2d(-3,0)),
      false));
      */
  
  // public Command getTestCommand(int testNumber){
    //return new ReefMoveToPosition(1,1,m_ladder,m_robotDrive,m_intake);
    //return new MoveToPosition(m_robotDrive, new Pose2d(5,5,new Rotation2d()), List.of(new Translation2d(5,0)), false)
    //.andThen(new MoveToPosition(m_robotDrive, new Pose2d(0,0,new Rotation2d()), List.of(), true));

    /*
    switch(testNumber){
      case 1: 
        return new MoveToPosition(
          m_robotDrive,
          new Pose2d(0.5, -0.5, new Rotation2d(0)),
          List.of(),
          false
        );
      case 2:
        return new ReefMoveToPosition(1, 1, m_ladder, m_robotDrive, m_intake);//reefPathMaker.createCommand(1, 1, m_ladder, m_robotDrive, m_intake);
      case 3:
        return new LadderMoveToPosition(m_ladder, Constants.LadderConstants.bottomStalkPosition);
      default:
        return new Command() {
          
        };
        
    }
        */
  // }
}
