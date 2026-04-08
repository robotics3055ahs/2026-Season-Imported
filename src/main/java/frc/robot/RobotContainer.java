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

import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems

  // The driver's controller
  private static final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  PowerDistribution m_PDP = new PowerDistribution(OIConstants.pdpPort, ModuleType.kRev);
  private final PathMaker m_PathMaker = new PathMaker();
  private static final VisionSubsystem m_robotVision = new VisionSubsystem();
  private static final PhotonCamera m_frontCamera = m_robotVision.getPhotonCamera();
  private static final ShooterSubsystem m_shooter = new ShooterSubsystem(m_driverController);
  private static final AnalogInput m_potADC = m_shooter.getPotADC();
  public static final AnalogPotentiometer m_potentiometer = m_shooter.getPotentiometer();
  public static final AHRS m_gyro = m_robotDrive.getGyro();
  public static final double originalRPM = PIDConstants.ShooterConstants.shooterSpeedRPM;
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
    // Toggle swinger: if currently out, schedule SwingInCommand; otherwise schedule SwingOutCommand.
    // new JoystickButton(m_driverController, 6).toggleOnTrue(new IntakeKrakenCommand(m_shooter));
    // new JoystickButton(m_driverController, 5).toggleOnTrue(new ShootCommand(m_shooter));
    new JoystickButton(m_driverController, 7).onTrue(new InstantCommand(() -> m_robotDrive.resetGyro()));
    new JoystickButton(m_driverController, 8).onTrue(new InstantCommand(() -> m_fieldRelative = !m_fieldRelative));
  }

  public void Drive(){
    // Calculate drivetrain commands from Joystick values
    m_visionForward = Math.abs(m_driverController.getRawAxis(OIConstants.driveForwardAxis)) > 0.005 ? 
      -(m_driverController.getRawAxis(OIConstants.driveForwardAxis)) * 8 : 0;
    m_visionStrafe = Math.abs(m_driverController.getRawAxis(OIConstants.driveStrafeAxis)) > 0.005 ? 
      -(m_driverController.getRawAxis(OIConstants.driveStrafeAxis)) * 8 : 0;
    m_visionTurn = Math.abs(m_driverController.getRawAxis(OIConstants.driveTurnAxis)) > 0.005 ? 
      -(m_driverController.getRawAxis(OIConstants.driveTurnAxis)) * DriveConstants.kMaxSpeedMetersPerSecond: 0;
  }
  
  public void VisionMoveToTarget() {
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
    
    // While button X is being pressed, auto align ANGLE to april tag
    if (targetVisible && m_driverController.getXButton()) {
      if (m_fieldRelative) {
        m_fieldRelative = false;
        wasFieldRelative = true;
      } else {
        wasFieldRelative = false;
      }
      double turnOutput = m_visionTurnPID.calculate(targetYaw, 0);
      m_visionTurn = Math.abs(turnOutput) > VisionConstants.VISION_TURN_OUTPUT_DEADBAND ? turnOutput : 0;
    }
    // While button Y is being pressed, auto align STRAFE & FORWARD to april tag
    if (targetVisible && m_driverController.getYButton()) {
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
    SmartDashboard.putBoolean("wasFieldRelative ?", wasFieldRelative);
  }

  public XboxController getDriverController(){
    return m_driverController;
  }

  public void initDashboard(){
    SmartDashboard.putNumber("Auto Selector", OIConstants.autoSelected);
    SmartDashboard.putNumber("Chosen Hub Target", VisionConstants.hubTargetID);
    // Changable constants that will be updated from updateConstants()
    SmartDashboard.putNumber("Speed", DriveConstants.kMaxSpeedMetersPerSecond);
    // PID for shooter
    SmartDashboard.putNumber(
      "Shooter kP", 
      PIDConstants.ShooterConstants.kP
    );
    SmartDashboard.putNumber(
      "Shooter kI", 
      PIDConstants.ShooterConstants.kI
    );
    SmartDashboard.putNumber(
      "Shooter kD", 
      PIDConstants.ShooterConstants.kD
    );
    SmartDashboard.putNumber("Want Shooter PID to change?",0);
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
    SmartDashboard.putBoolean("Is Shooter on?", m_shooter.isShooterOn());
    SmartDashboard.putBoolean("Is Kraken On?", m_shooter.isKrakenOn());
    SmartDashboard.putData("M_potADC", m_potADC);
    SmartDashboard.putNumber(
      "Potentiometer Accounted for Offset", 
      m_potentiometer.get() - PIDConstants.IntakeSwingerConstants.potOffset
    );
    SmartDashboard.putData(
      "Potentiometer Without Offset", 
      m_potentiometer
    );
    SmartDashboard.putNumber("forward", m_visionForward);
    SmartDashboard.putNumber("strafe", m_visionStrafe);
    SmartDashboard.putNumber("turn", m_visionTurn);
    SmartDashboard.putBoolean("Top Limit Switch", m_shooter.getTopLimitSwitch());
    SmartDashboard.putBoolean("Bottom Limit Switch", m_shooter.getBottomLimitSwitch());
    SmartDashboard.putNumber("Shooter RPM", m_shooter.getShooterRPM());
    SmartDashboard.putNumber("Calculated Current Position", m_shooter.getCalculatedCurrentPosition());
    SmartDashboard.putNumber("Target RPM", PIDConstants.ShooterConstants.shooterSpeedRPM);
  }

  public void updateConstants(){
    // Speed updater
    double tempSpeed = SmartDashboard.getNumber("Speed", 5);
    DriveConstants.kMaxSpeedMetersPerSecond = tempSpeed;
    // Auto updater
    int tempAuto = (int) SmartDashboard.getNumber("Auto Selector", -1);
    OIConstants.autoSelected = tempAuto;
    // Hub updater
    int tempHub = (int) SmartDashboard.getNumber("Chosen Hub Target", 1);
    VisionConstants.hubTargetID = tempHub;
    if(SmartDashboard.getNumber("Want Shooter PID to change?", 0) == 1){
      double tempKP = SmartDashboard.getNumber(
        "Shooter kP", 
        PIDConstants.ShooterConstants.kP
      );
      double tempKI = SmartDashboard.getNumber(
        "Shooter kI", 
        PIDConstants.ShooterConstants.kI
      );
      double tempKD = SmartDashboard.getNumber(
        "Shooter kD", 
        PIDConstants.ShooterConstants.kD
      );
      m_shooter.updateShooterPID(tempKP, tempKI, tempKD);
      SmartDashboard.putNumber("Want Shooter PID to change?",0);
    }
  }

  public void periodic() {
    Drive();
    m_shooter.swingerHandler();
    m_shooter.shooterHandler();
    m_shooter.krakenHandler();
    updateDashboard();
    updateConstants();
    if(m_robotVision.cameraConnected){
      VisionMoveToTarget();
    }
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
