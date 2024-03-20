// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.opencv.photo.Photo;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private final PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
  private final Transform3d robotToCam = new Transform3d(new Translation3d(0.958, 0.285, 0.4), new Rotation3d(0,Math.toRadians(20),Math.toRadians(0))); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

  // Construct PhotonPoseEstimator
  PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return photonPoseEstimator.update();
  }
  public PhotonCamera getCamera() {
    return camera;
  }
  public Command getAmpAlignCommand() {
    return Commands.run(null, null);
  }
  @Override
  public void periodic() {

  }
}
