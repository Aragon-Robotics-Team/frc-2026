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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;

public class Vision extends SubsystemBase {
  private PhotonCamera m_leftCam = new PhotonCamera(VisionConstants.kLeftCamName);
  private PhotonPoseEstimator m_leftPhotonEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout, VisionConstants.kLeftRobotToCam);
  
  private PhotonCamera m_rightCam = new PhotonCamera(VisionConstants.kRightCamName);
  private PhotonPoseEstimator m_rightPhotonPoseEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout, VisionConstants.kRightRobotToCam);

  private EstimateConsumer m_estimateConsumer;

  private Optional<EstimatedRobotPose> m_leftVisionEstimate = Optional.empty();
  private Optional<EstimatedRobotPose> m_rightVisionEstimate = Optional.empty();

  /** Creates a new Vision. */
  public Vision(EstimateConsumer estimateConsumer) {
    m_estimateConsumer = estimateConsumer;
  }

  public Matrix<N3, N1> updateStandardDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, PhotonPoseEstimator estimator, Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevs) {
    Matrix<N3, N1> curStdDevs = singleTagStdDevs;

    if (estimatedPose.isPresent()) {
      //pose present. Start running heuristic
      Matrix<N3, N1> estStdDevs = singleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      //precalculation - count tags found, calculate avg distance
      for (PhotonTrackedTarget tgt : targets) {
        Optional<Pose3d> tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) 
          continue;
        numTags ++;
        avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        //No tags visible. Default to single-tag std devs
        curStdDevs = singleTagStdDevs;
      }
      else {
        //Tags visible, run full heuristic
        avgDist /= numTags;
        
        if (numTags > 1)
          estStdDevs = multiTagStdDevs;
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else {
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
      }
    }
    return curStdDevs;  
  }

  public void estimatePoseFromResults(PhotonCamera cam, Optional<EstimatedRobotPose> estimate, PhotonPoseEstimator estimator, Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevs) {
    for (var result : cam.getAllUnreadResults()) {
      estimate = estimator.estimateCoprocMultiTagPose(result);

      if (estimate.isEmpty()) {
        estimate = estimator.estimateLowestAmbiguityPose(result);
      }

      if (estimate.isPresent()) {
        m_estimateConsumer.accept(estimate.get().estimatedPose.toPose2d(), estimate.get().timestampSeconds, updateStandardDevs(estimate, result.getTargets(), estimator, singleTagStdDevs, multiTagStdDevs));
      }      
    }  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    estimatePoseFromResults(m_leftCam, m_leftVisionEstimate, m_leftPhotonEstimator, VisionConstants.kLeftSingleTagStdDevs, VisionConstants.kLeftMultiTagStdDevs);
    estimatePoseFromResults(m_rightCam, m_rightVisionEstimate, m_rightPhotonPoseEstimator, VisionConstants.kRightSingleTagStdDevs, VisionConstants.kRightMultiTagStdDevs);

  }

  @FunctionalInterface
  public static interface EstimateConsumer {
      public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStandardDevs);
  }
}
