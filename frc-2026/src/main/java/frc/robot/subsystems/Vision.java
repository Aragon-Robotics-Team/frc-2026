// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;

public class Vision extends SubsystemBase {
  private PhotonCamera m_cam = new PhotonCamera(VisionConstants.kCamName);
  private PhotonPoseEstimator m_photonEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout, VisionConstants.kRobotToCam);

  private EstimateConsumer m_estimateConsumer;

  private Optional<EstimatedRobotPose> m_visionEstimate = Optional.empty();

  /** Creates a new Vision. */
  public Vision(EstimateConsumer estimateConsumer) {
    m_estimateConsumer = estimateConsumer;
  }

  public void updateStandardDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

  }

  public Matrix<N3, N1> getStandardDevs() {
    return null;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      for (var result : m_cam.getAllUnreadResults()) {
        m_visionEstimate = m_photonEstimator.estimateCoprocMultiTagPose(result);
        if (m_visionEstimate.isEmpty()) {
          m_visionEstimate = m_photonEstimator.estimateLowestAmbiguityPose(result);
        }

        if (m_visionEstimate.isPresent()) {
          m_estimateConsumer.accept(m_visionEstimate.get().estimatedPose.toPose2d(), m_visionEstimate.get().timestampSeconds, getStandardDevs());
        }      
      }  

  }

  @FunctionalInterface
  public static interface EstimateConsumer {
      public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStandardDevs);
  }
}
