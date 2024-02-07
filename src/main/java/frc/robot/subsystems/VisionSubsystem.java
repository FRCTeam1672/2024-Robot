// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class VisionSubsystem extends SubsystemBase {
  // TODO Change the camera
  private PhotonCamera camera = new PhotonCamera("OV2981_CAMERA");
  private PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      camera,
      // TODO add the transform for our camera
      new Transform3d());
  private Pose3d pose;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
  }

  @Override
  public void periodic() {
    // obtain the latest pose, and feed it into our pose estimator
    if (camera.getLatestResult().getMultiTagResult().estimatedPose.isPresent) {
      Optional<EstimatedRobotPose> update = poseEstimator.update();
      if (update.isPresent()) {
        pose = update.get().estimatedPose;
        Robot.getInstance().getRobotContainer().getSwerveSubsystem().addVisionReading(pose.toPose2d(), update.get().timestampSeconds);
      }

    }
  }

  public Pose3d getPose3D() {
    return pose;
  }
}
