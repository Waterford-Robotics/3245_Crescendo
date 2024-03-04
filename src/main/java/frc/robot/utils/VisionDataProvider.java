// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

/** A VisionDataProvider provides vision data from a PhotonVision camera. */
public class VisionDataProvider {

  private PhotonCamera m_camera;
  private PhotonPoseEstimator m_poseEstimator;
  
  /**
   * Creates a new VisionDataProvider.
   * 
   * @param cameraName The name of the camera.
   * @param robotToCameraTransform The 3D transform from the robot's center to the camera.
   * @param field The AprilTag field layout.
   */
  public VisionDataProvider(String cameraName, Transform3d robotToCameraTransform, AprilTagFields field) {
    m_camera = new PhotonCamera(cameraName);
    m_poseEstimator = new PhotonPoseEstimator(field.loadAprilTagLayoutField(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera, robotToCameraTransform);
    m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
  }

  /**
   * Creates a new VisionDataProvider.
   * 
   * @param cameraName The name of the camera.
   * @param robotToCameraTransform The 3D transform from the robot's center to the camera.
   * @param field The AprilTag field layout.
   */
  public VisionDataProvider(String cameraName, Transform3d robotToCameraTransform, AprilTagFieldLayout field) {
    m_camera = new PhotonCamera(cameraName);
    m_poseEstimator = new PhotonPoseEstimator(field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera, robotToCameraTransform);
    m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
  }

  /**
   * Gets the camera providing the vision data.
   * @return The camera.
   */
  public PhotonCamera getCamera() {
    return m_camera;
  }

  /**
   * Gets the targets the camera currently sees. It is stronly recommended that you call this method only once per
   * update to guarantee the fetched targets are from the same vision snapshot.
   * @return A list of the targets the camera currently sees.
   */
  public List<PhotonTrackedTarget> getTargets() {
    PhotonPipelineResult result = m_camera.getLatestResult();
    if (result.hasTargets()) {
      return result.getTargets();
    } else {
      return List.of();
    }
  }

  /**
   * Gets the transform from the robot's center to the camera.
   * @return A {@link Transform3d} representing the transform from the robot's center to the camera.
   */
  public Transform3d getRobotToCameraTransform() {
    return m_poseEstimator.getRobotToCameraTransform();
  }

  /**
   * Sets the transform from the robot's center to the camera.
   * @param robotToCameraTransform The new robot-to-camera transform.
   */
  public void setRobotToCameraTransform(Transform3d robotToCameraTransform) {
    m_poseEstimator.setRobotToCameraTransform(robotToCameraTransform);
  }

  /**
   * Get the field layout of the AprilTags being used to estimate the robot's pose.
   * @return An {@link AprilTagFieldLayout} representing the layout of the AprilTags.
   */
  public AprilTagFieldLayout getFieldLayout() {
    return m_poseEstimator.getFieldTags();
  }

  /**
   * Get the estimated global pose of the robot using the vision data, if it exists, using the previous pose to help
   * refine the estimate. If the vision system can't estimate a robot pose when this is called, the result will be
   * empty.
   * @param previousEstimatedRobotPose The previously-estimated robot pose.
   * @return The newly estimated robot pose if an estimate could be made, otherwise Optional.empty().
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d previousEstimatedRobotPose) {
    m_poseEstimator.setReferencePose(previousEstimatedRobotPose);
    return m_poseEstimator.update();
  }
}
