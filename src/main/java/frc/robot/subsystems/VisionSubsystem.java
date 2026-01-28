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
  /** Creates a new ExampleSubsystem. */
  
  static AprilTagFieldLayout tagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
  public boolean cameraConnected = true; 
  
  static Transform3d robotToFrontCamera = new Transform3d(new Translation3d(0.305,-0.305,0.1), new Rotation3d(0,0,0));
  static PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(tagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToFrontCamera);
  static PhotonPipelineResult frontCameraResult;
  static List<Integer> reefIds= List.of(6,7,8,9,10,11,17,18,19,20,21,22);
  DoublePublisher xReefTransform = NetworkTableInstance.getDefault().getTable("reefTransforms").getDoubleTopic("xReefTransform").publish();
  DoublePublisher yReefTransform = NetworkTableInstance.getDefault().getTable("reefTransforms").getDoubleTopic("yReefTransform").publish();
  DoublePublisher rotReefTransform = NetworkTableInstance.getDefault().getTable("reefTransforms").getDoubleTopic("rotReefTransform").publish(); 
  static PhotonCamera frontCamera = new PhotonCamera("FrontCamera");
  static PhotonTrackedTarget cameraTarget;
  
  public VisionSubsystem(){
    cameraConnected = true;
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
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

  public EstimatedRobotPose getEstimatedGlobalPose() {
    if(frontCamera.isConnected()){
      var estimatedRobotPose = poseEstimator.update(frontCameraResult);
      if(estimatedRobotPose.isPresent()){
        poseEstimator.setLastPose(estimatedRobotPose.get().estimatedPose);
        return estimatedRobotPose.get();
      }
    }
    return null;
  }

  public Pose2d getEstimatedLocalPose(){
    if(frontCamera.isConnected()){
      List<PhotonTrackedTarget> targetResults = frontCameraResult.getTargets();
      if(!targetResults.isEmpty()){
        Transform3d targetToRobot = targetResults.get(0).getBestCameraToTarget().inverse().plus(robotToFrontCamera);
        return new Pose2d(targetToRobot.getTranslation().toTranslation2d(), targetToRobot.getRotation().toRotation2d());
      }
    }
    return null;
  }

  public Transform2d getTargetTransform(int targetId) {
    updateCamera();
    if (frontCamera.isConnected() && frontCameraResult != null) {
        List<PhotonTrackedTarget> targets = frontCameraResult.getTargets();
        for (PhotonTrackedTarget target : targets) {
            if (target.getFiducialId() == targetId) {
                Transform3d targetToRobot = target.getBestCameraToTarget().inverse().plus(robotToFrontCamera);
                return new Transform2d(targetToRobot.getTranslation().toTranslation2d(), targetToRobot.getRotation().toRotation2d());
            }
        }
    }
    return null;
 }

  public Transform2d getReefTransform(){
    updateCamera();
    
    if(frontCamera.isConnected()){
      if(frontCameraResult != null){
        List<PhotonTrackedTarget> targetResults = frontCameraResult.getTargets();
        if(targetResults != null){
          if(targetResults.size() != 0){
            if(reefIds.contains(targetResults.get(0).fiducialId) && targetResults.size() != 0){
              Transform3d robotToTarget = targetResults.get(0).getBestCameraToTarget().plus(new Transform3d(new Translation3d(), new Rotation3d(0,0,-Math.PI)));
              rotReefTransform.set(robotToTarget.getRotation().toRotation2d().getDegrees());
              xReefTransform.set(robotToTarget .getX());
              yReefTransform.set(robotToTarget.getY());
              return new Transform2d(robotToTarget.getTranslation().toTranslation2d(), robotToTarget.getRotation().toRotation2d());
            }
          }
        }
      }
    }
    return null;
  }

  public Pose2d getPoseToTarget(int targetId) {
    updateCamera();
    if (frontCamera.isConnected() && frontCameraResult != null) {
        List<PhotonTrackedTarget> targets = frontCameraResult.getTargets();
        for (PhotonTrackedTarget target : targets) {
            if (target.getFiducialId() == targetId) {
                Transform3d targetToRobot = target.getBestCameraToTarget().inverse().plus(robotToFrontCamera);
                Pose2d targetPose = new Pose2d(
                    targetToRobot.getTranslation().toTranslation2d(),
                    targetToRobot.getRotation().toRotation2d()
                );
                // Calculate the position 1 meter away from the target
                double angle = targetPose.getRotation().getRadians();
                double offsetX = Math.cos(angle) * 1.0; // 1 meter offset
                double offsetY = Math.sin(angle) * 1.0;
                return new Pose2d(
                    targetPose.getX() - offsetX,
                    targetPose.getY() - offsetY,
                    targetPose.getRotation()
                );
            }
        }
    }
    return null;
  }

  /**
   * Returns a list of Translation2d objects representing detected positions.
   * This method fetches positions from the vision system.
   */
  public List<Translation2d> getDetectedTranslations2d(int m_targetID) {
    updateCamera(); // Ensure the latest camera data is used
    List<Translation2d> positions = new ArrayList<>();
    if (frontCamera.isConnected() && frontCameraResult != null) {
      List<PhotonTrackedTarget> targets = frontCameraResult.getTargets();
      for (PhotonTrackedTarget target : targets) {
        if (target.getFiducialId() == m_targetID){
          Transform3d targetToRobot = target.getBestCameraToTarget().inverse().plus(robotToFrontCamera);
          positions.add(targetToRobot.getTranslation().toTranslation2d());
        }
      }
    }
    return positions;
  }
}
