// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.NeoResponse;
import frc.robot.commands.driveCommands.MoveToPosition;
import frc.robot.commands.PathMaker;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems

  private static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  PowerDistribution m_PDP = new PowerDistribution(OIConstants.pdpPort, ModuleType.kRev);
  private final PathMaker m_PathMaker = new PathMaker();
  private static final VisionSubsystem m_robotVision = new VisionSubsystem();
  private static final PhotonCamera m_frontCamera = m_robotVision.getPhotonCamera();
  private static final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private static final AnalogInput m_potADC = m_shooter.getPotADC();
  public static final AnalogPotentiometer m_potentiometer = m_shooter.getPotentiometer();
  public static final ADXRS450_Gyro m_gyro = m_robotDrive.getGyro();
  private boolean m_fieldRelative = true;
  public ShuffleboardTab tab;
  // vision drive values
  double m_visionForward;
  double m_visionStrafe;
  double m_visionTurn;
  // vision PID
  PIDController m_visionTurnPID = new PIDController(
    VisionConstants.VISION_TURN_kP, 
    VisionConstants.VISION_TURN_kI, 
    VisionConstants.VISION_TURN_kD
  );
  PIDController m_visionForwardPID = new PIDController(
    VisionConstants.VISION_FORWARD_kP, 
    VisionConstants.VISION_FORWARD_kI, 
    VisionConstants.VISION_FORWARD_kD
  );
  PIDController m_visionStrafePID = new PIDController(
    VisionConstants.VISION_STRAFE_kP, 
    VisionConstants.VISION_STRAFE_kI, 
    VisionConstants.VISION_STRAFE_kD
  );
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
    new RunCommand(
      () -> 
        m_robotDrive.drive(
        m_visionForward, 
        m_visionStrafe, 
        m_visionTurn,
        m_fieldRelative),
    m_robotDrive)
  );
}
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, 5).toggleOnTrue(new IntakeCommand(m_shooter));
    new JoystickButton(m_driverController, 6).toggleOnTrue(new ShootCommand(m_shooter));
    new JoystickButton(m_driverController, 7).onTrue(new InstantCommand(() -> m_robotDrive.resetGyro()));
    new JoystickButton(m_driverController, 8).onTrue(new InstantCommand(() -> m_fieldRelative = !m_fieldRelative));
  }
  
  public void VisionMoveToTarget() {
    // Calculate drivetrain commands from Joystick values
    m_visionForward = Math.abs(m_driverController.getRawAxis(OIConstants.driveForwardAxis)) > 0.05 ? -(m_driverController.getRawAxis(OIConstants.driveForwardAxis)) * DriveConstants.kMaxSpeedMetersPerSecond : 0;
    m_visionStrafe = Math.abs(m_driverController.getRawAxis(OIConstants.driveStrafeAxis)) > 0.05 ? -(m_driverController.getRawAxis(OIConstants.driveStrafeAxis)) * DriveConstants.kMaxSpeedMetersPerSecond : 0;
    m_visionTurn = Math.abs(m_driverController.getRawAxis(OIConstants.driveTurnAxis)) > 0.3 ? -m_driverController.getRawAxis(OIConstants.driveTurnAxis) * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond: 0;
    // temp field centric boolean
    boolean wasFieldRelative = false;
    // Read in relevant data from the Camera
    int m_targetID = VisionConstants.hubTargetID;
    boolean targetVisible = false;
    double targetYaw = 0.0;
    double targetRange = 0.0;
    PhotonTrackedTarget cameraTarget = m_robotVision.getCameraTarget();
    if (cameraTarget != null) {
      if (cameraTarget.getFiducialId() == m_targetID) {
        // Found Desired Tag, record its information
        targetYaw = cameraTarget.getYaw();
        targetRange =
        PhotonUtils.calculateDistanceToTargetMeters(
          VisionConstants.cameraHeightMeters, // Height of the camera off the ground on the (meters)
          VisionConstants.hubHeightMeters, // Height of the center of the april tag off the ground (meters)
          Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
          Units.degreesToRadians(cameraTarget.getPitch())
        );
        SmartDashboard.putNumber("target range",targetRange);
        targetVisible = true;
      }
    }
    
    // While button A is being pressed, auto align ANGLE to april tag
    if (targetVisible && m_driverController.getAButton()) {
      if (m_fieldRelative) {
        m_fieldRelative = false;
        wasFieldRelative = true;
      } else {
        wasFieldRelative = false;
      }
      double turnOutput = m_visionTurnPID.calculate(targetYaw, 0);
      m_visionTurn = Math.abs(turnOutput) > VisionConstants.VISION_TURN_OUTPUT_DEADBAND ? turnOutput : 0;
    }
    // While button B is being pressed, auto align STRAFE & FORWARD to april tag
    if (targetVisible && m_driverController.getBButton()) {
      if (m_fieldRelative) {
        m_fieldRelative = false;
        wasFieldRelative = true;
      } else {
        wasFieldRelative = false;
      }
      double strafeOutput = m_visionStrafePID.calculate(targetYaw, 0);
      m_visionStrafe = Math.abs(strafeOutput) > VisionConstants.VISION_STRAFE_OUTPUT_DEADBAND ? strafeOutput : 0;
      double forwardOutput = -m_visionForwardPID.calculate(targetRange, -1);
      m_visionForward = Math.abs(forwardOutput) > VisionConstants.VISION_FORWARD_OUTPUT_DEADBAND ? forwardOutput : 0;
    }
    if (wasFieldRelative){
      SmartDashboard.putBoolean("Field Relative?", m_fieldRelative);
      m_fieldRelative = true;
    }
    // Command drivetrain motors based on target speeds
    SmartDashboard.putNumber("forward", m_visionForward);
    SmartDashboard.putNumber("strafe", m_visionStrafe);
    SmartDashboard.putNumber("turn", m_visionTurn);
    SmartDashboard.putBoolean("wasFieldRelative ?", wasFieldRelative);
  }

  public void initDashboard(){
    SmartDashboard.putNumber("Auto Selector", OIConstants.autoSelected);
    SmartDashboard.putNumber("Chosen Hub Target", VisionConstants.hubTargetID);
    // Changable constants that will be updated from updateConstants()
    SmartDashboard.putNumber("Speed", DriveConstants.kMaxSpeedMetersPerSecond);
    SmartDashboard.putNumber("Forward Axis", (int) OIConstants.driveForwardAxis);
    SmartDashboard.putNumber("Strafe Axis", (int) OIConstants.driveStrafeAxis);
    SmartDashboard.putNumber("Turn Axis", (int) OIConstants.driveTurnAxis);
    // PID for swinger
    SmartDashboard.putNumber(
      "Swinger kP", 
      PIDConstants.IntakeSwingerConstants.kP
    );
    SmartDashboard.putNumber(
      "Swinger kI", 
      PIDConstants.IntakeSwingerConstants.kI
    );
    SmartDashboard.putNumber(
      "Swinger kD", 
      PIDConstants.IntakeSwingerConstants.kD
    );
  }

  public void updateDashboard(){
    if(m_frontCamera.isConnected()){
      SmartDashboard.putBoolean("Camera Connected", true);
      PhotonTrackedTarget m_cameraTarget = m_robotVision.getCameraTarget();
      if(m_cameraTarget != null){
        SmartDashboard.putNumber("Camera Target ID", m_cameraTarget.getFiducialId());
        SmartDashboard.putNumber("Camera Target Pitch", m_cameraTarget.getPitch());
        SmartDashboard.putNumber("Camera Target Yaw", m_cameraTarget.getYaw());
        SmartDashboard.putNumber("Camera Target Area", m_cameraTarget.getArea());
        SmartDashboard.putString("Camera Target", m_cameraTarget.toString());
        SmartDashboard.putNumber("Camera Skew", m_cameraTarget.getSkew());
      } else {
        SmartDashboard.putString("Camera Target", "No Target");
      }
    } else {
      SmartDashboard.putBoolean("Camera Connected", false);
    }
    SmartDashboard.putBoolean("Field Relative?", m_fieldRelative);
    SmartDashboard.putData("PDP Data", m_PDP);
    SmartDashboard.putData("Gyro", m_gyro);
    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
    SmartDashboard.putData("Drive Subsystem", m_robotDrive);
    SmartDashboard.putData("M_potADC", m_potADC);
    SmartDashboard.putData("Potentiometer", m_potentiometer);
  }

  public void updateConstants(){
    // Speed updater
    double tempSpeed = SmartDashboard.getNumber("Speed", 5);
    DriveConstants.kMaxSpeedMetersPerSecond = tempSpeed;
    // Controller axis updater
    // int tempForwardAxis = (int) SmartDashboard.getNumber("Forward Axis", 1);
    // int tempStrafeAxis = (int) SmartDashboard.getNumber("Strafe Axis", 0);
    // int tempTurnAxis = (int) SmartDashboard.getNumber("Turn Axis", 4);
    // OIConstants.driveForwardAxis = tempForwardAxis;
    // OIConstants.driveStrafeAxis = tempStrafeAxis;
    // OIConstants.driveTurnAxis = tempTurnAxis;
    // Auto updater
    int tempAuto = (int) SmartDashboard.getNumber("Auto Selector", -1);
    OIConstants.autoSelected = tempAuto;
    // Hub updater
    int tempHub = (int) SmartDashboard.getNumber("Chosen Hub Target", 1);
    VisionConstants.hubTargetID = tempHub;
    // Swinger PID
    double tempKP = SmartDashboard.getNumber(
      "Swinger kP", 
      PIDConstants.IntakeSwingerConstants.kP
    );
    double tempKI = SmartDashboard.getNumber(
      "Swinger kI", 
      PIDConstants.IntakeSwingerConstants.kI
    );
    double tempKD = SmartDashboard.getNumber(
      "Swinger kD", 
      PIDConstants.IntakeSwingerConstants.kD
    );
    m_shooter.updateSwingerPID(tempKP, tempKI, tempKD);
  }

  public void periodic() {
    updateDashboard();
    updateConstants();
    VisionMoveToTarget();
    m_robotVision.updateCamera();
    //m_robotDrive.changeMaxSpeed(m_driverRJoystick.getRawAxis(0));    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    int autoNumber = OIConstants.autoSelected;
    switch(autoNumber){
      case 0:
        return new InstantCommand();
      default:
        return new InstantCommand();
    }
  }
}
