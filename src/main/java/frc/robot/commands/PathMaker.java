// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class PathMaker {
    private Pose2d m_finalPose;  
    private Pose2d m_localPose;
    private List<Translation2d> m_transitionPoses;
    final DriveSubsystem m_drive = null;
    private Command commandSequence;

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
    public final Command createPath(DriveSubsystem m_drive, Pose2d finalPose, List<Translation2d> transitionPoses, Boolean globalPosBoolean){
        m_transitionPoses = transitionPoses;
        m_finalPose = finalPose;
        
        Trajectory robotTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0,0, new Rotation2d(0)),
                //Pass in points the robot should go through          
                m_transitionPoses,
                //final pose of the robot
                m_finalPose,
                m_config);

        ProfiledPIDController thetaController =
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 5, 1, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);  

        if(!globalPosBoolean){
            m_drive.setRelativePose(m_drive.getPose());
        }

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                robotTrajectory,
                globalPosBoolean == true ? m_drive::getPose : m_drive::getRelativePose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 5, 1),
                new PIDController(AutoConstants.kPYController, 5, 1),
                thetaController,
                m_drive::setModuleStates,
                m_drive);
        commandSequence = Commands.sequence(
            //new InstantCommand(() -> m_drive.resetOdometry(robotTrajectory.getInitialPose())), 
            swerveControllerCommand,
            new InstantCommand(() -> m_drive.drive(0, 0, 0, false)));
        return commandSequence;
    } 
}
