// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/** An example command that uses an example subsystem. */
public class MoveToPosition extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private Pose2d m_finalPose;  
  private List<Translation2d> m_transitionPoses;
  private boolean m_globalPosBoolean;
  private DriveSubsystem m_drive = null;
  SwerveControllerCommand swerveControllerCommand;
    private final TrajectoryConfig m_config =
    new TrajectoryConfig(
      AutoConstants.kMaxAutoSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);

    /**
    * @return Returns a path that the robot will follow
    * @param m_drive Main Drive subsystem
    * @param finalPose Final robot pose
    * @param transitionPoses List of translations the robot will pass through (List.of())
    * @param globalPosBoolean Are the provided points local to the robot (False) or the field (True)
    */
  
  public MoveToPosition(DriveSubsystem drive, Pose2d finalPose, List<Translation2d> transitionPoses, Boolean globalPosBoolean) {
    m_drive = drive;
    m_transitionPoses = transitionPoses;
    m_finalPose = finalPose;
    m_globalPosBoolean = globalPosBoolean;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    Trajectory robotTrajectory = 
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0,0, new Rotation2d(0)),
                List.of(),
                m_finalPose,                //final pose of the robot
                m_config);

        ProfiledPIDController thetaController =
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 0.5, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        if(!m_globalPosBoolean){
            m_drive.setRelativePose(m_drive.getPose());
        }

        swerveControllerCommand =
            new SwerveControllerCommand(
                robotTrajectory,
                m_globalPosBoolean == true ? m_drive::getPose : m_drive::getRelativePose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(5,5,0.5),
                new PIDController(5,5,0.5),
                thetaController,
                m_drive::setModuleStates,
                m_drive);
        swerveControllerCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(swerveControllerCommand.isFinished());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveControllerCommand.end(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(swerveControllerCommand.isFinished());
    return swerveControllerCommand.isFinished();
  }
}
