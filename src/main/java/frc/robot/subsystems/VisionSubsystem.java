// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.ArrayList;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class VisionSubsystem extends SubsystemBase {
  static AprilTagFieldLayout tagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  public boolean cameraConnected = true; 
  static PhotonPipelineResult frontCameraResult;
  static PhotonCamera frontCamera = new PhotonCamera("FrontCamera");
  static PhotonTrackedTarget cameraTarget;
  
  public VisionSubsystem(){
    cameraConnected = true;
    try{
      frontCamera = new PhotonCamera("FrontCamera");
    } catch(ArithmeticException exception){
      cameraConnected = false;
    }
  }

  public PhotonCamera getPhotonCamera() {
    return frontCamera;
  }
  public PhotonTrackedTarget getCameraTarget(){
    return cameraTarget;
  }

  public void updateCamera(){
    if(frontCamera.isConnected()){
      List<PhotonPipelineResult> cameraResults = frontCamera.getAllUnreadResults();
      if(cameraResults.size() != 0){
        frontCameraResult = cameraResults.get(cameraResults.size() - 1);
        cameraTarget = frontCameraResult.getBestTarget();
      }
    }
  }
}
