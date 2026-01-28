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
  private final PowerDistribution m_PDP = new PowerDistribution();
  private final PathMaker m_PathMaker = new PathMaker();
  private final VisionSubsystem m_robotVision = new VisionSubsystem();
  private final PhotonCamera m_Camera = m_robotVision.getPhotonCamera();
  private boolean m_fieldRelative = true;
  public ShuffleboardTab tab;
  // Target varialbe for vision subsystem
  Pose2d poseToTarget;
  Transform2d transformToTarget;
  List<Translation2d> detectedTranslations2d;

public boolean driverDriveControlEnabled = true;

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
	new JoystickButton(m_driverController, 5).whileTrue(
        new InstantCommand(() -> robotMoveToAprilTag())
    );
	
	/*if(poseToTarget != null && detectedTranslations2d != null){
		new MoveToPosition(
			m_robotDrive,
			poseToTarget, // Pose to target from VisionSubsystem
			detectedTranslations2d, // List of Translation2d from VisionSubsystem
			m_fieldRelative // Field-relative boolean
		);
	}*/
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_driverController.getRawButton(0)).onTrue();

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b(0).whileTrue();
  }

  public void initDashboard(){
    SmartDashboard.putNumber("Auto Selector", 0);
  }

  public void robotMoveToAprilTag(){
	// Calculate drivetrain commands from Joystick values
    double forward = Math.abs(m_driverController.getRawAxis(1)) > 0.05 ? -(m_driverController.getRawAxis(1)) * DriveConstants.kMaxSpeedMetersPerSecond : 0;
    double strafe = Math.abs(m_driverController.getRawAxis(0)) > 0.05 ? -(m_driverController.getRawAxis(0)) * DriveConstants.kMaxSpeedMetersPerSecond : 0;
    double turn = Math.abs(m_driverController.getRawAxis(2)) > 0.3 ? -m_driverController.getRawAxis(2) * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond: 0;
    
    // Vision movement speeds
    double VISION_TURN_kP = 0.03; // Proportional turning constant to minimize yaw error
    double VISION_STRAFE_kP = 0.1; // Proportional strafe constant to minimize distance error

    // Read in relevant data from the Camera
    boolean targetVisible = false;
    double targetYaw = 0.0;
    double targetRange = 0.0;
    PhotonCamera camera = m_robotVision.getPhotonCamera();
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        // At least one AprilTag was seen by the camera
        for (var target : result.getTargets()) {
          if (target.getFiducialId() == 7) {
            // Found Tag 7, record its information
            targetYaw = target.getYaw();
            targetRange =
            PhotonUtils.calculateDistanceToTargetMeters(
            0.5, // Measured with a tape measure, or in CAD.
            1.435, // From 2024 game manual for ID 7
            Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
            Units.degreesToRadians(target.getPitch()));
            targetVisible = true;
          }
        }
      }
    }
    
    // Auto-align when requested
    if (m_driverController.getAButton() && targetVisible) {
      // Driver wants auto-alignment to tag 7
      // And, tag 7 is in sight, so we can turn toward it.
      // Override the driver's turn and fwd/rev command with an automatic one
      // That turns toward the tag, and gets the range right.
      turn = (-targetYaw) * VISION_TURN_kP * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond;
      forward = (-targetRange) * VISION_STRAFE_kP * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond;
    }
    
    // Command drivetrain motors based on target speeds
    m_robotDrive.drive(forward, strafe, turn, m_fieldRelative);
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
    SmartDashboard.putData("PDP Data", m_PDP);
    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
    SmartDashboard.putData("Drive Subsystem", m_robotDrive);
  }

  public void updateVisionTarget(){
	int m_targetID = Constants.VisionConstants.hubTargetID;
    Pose2d estimatedPose = m_robotVision.getEstimatedLocalPose();
    if(estimatedPose != null){
      SmartDashboard.putNumber("Vision X", estimatedPose.getX());
      SmartDashboard.putNumber("Vision Y", estimatedPose.getY());
      SmartDashboard.putNumber("Vision Rot", estimatedPose.getRotation().getDegrees());
      SmartDashboard.putNumber("Vision Target:", m_targetID);
    }
    m_robotVision.updateCamera();
    poseToTarget = m_robotVision.getPoseToTarget(m_targetID);
    if(poseToTarget != null){
      SmartDashboard.putNumber("Pose to Target X", poseToTarget.getX());
      SmartDashboard.putNumber("Pose to Target Y", poseToTarget.getY());
      SmartDashboard.putNumber("Pose to Target Rot", poseToTarget.getRotation().getDegrees());
    }
    transformToTarget = m_robotVision.getTargetTransform(m_targetID);
    if(transformToTarget != null){
      SmartDashboard.putNumber("Transform to Target X", transformToTarget.getTranslation().getX());
      SmartDashboard.putNumber("Transform to Target Y", transformToTarget.getTranslation().getY());
      SmartDashboard.putNumber("Transform to Target Rot", transformToTarget.getRotation().getDegrees());
    }
    detectedTranslations2d = m_robotVision.getDetectedTranslations2d(m_targetID);
    if(detectedTranslations2d != null){
        for(int i = 0; i < detectedTranslations2d.size(); i++){
            SmartDashboard.putNumber("Detected Position " + i + " X", detectedTranslations2d.get(i).getX());
            SmartDashboard.putNumber("Detected Position " + i + " Y", detectedTranslations2d.get(i).getY());
        }
    }
  }
  
  /*public void photonVisionService() {
    
  }
    */
  public void periodic() {
    updateDashboard();
    updateVisionTarget();
    m_robotVision.updateCamera();
    //m_robotDrive.changeMaxSpeed(m_driverRJoystick.getRawAxis(0));    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(int autoNumber) {
    return new RunCommand(()->m_robotDrive.drive(-1,0,0, true), m_robotDrive).withTimeout(3)
		.andThen(()->System.out.println("Auto Step 1 Complete")).withTimeout(2)
		.andThen(()->m_robotDrive.drive(0,-1,0,true), m_robotDrive).withTimeout(3)
		.andThen(()->System.out.println("Auto Step 2 Complete")).withTimeout(2)
		.andThen(()->m_robotDrive.drive(0,0,1, true), m_robotDrive).withTimeout(3)
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
