package frc.robot.subsystems.apriltagvision;

import static java.lang.System.arraycopy;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class AprilTagVisionIOReal implements AprilTagVisionIO {

  // Left Side Camera
  private final PhotonCamera leftCam;
  private final PhotonPoseEstimator leftPhotonPoseEstimator;

  // Right Side Camera
  private final PhotonCamera rightCam;
  private final PhotonPoseEstimator rightPhotonPoseEstimator;

  private Pose3d[] poseArray = new Pose3d[2];
  private double[] timestampArray = new double[2];
  private double[] visionStdArray = new double[6];
  private double[] latencyArray = new double[2];
  private int count = 0;

  public AprilTagVisionIOReal() {
    // Left Side Camera
    leftCam = new PhotonCamera("left");
    leftPhotonPoseEstimator =
        new PhotonPoseEstimator(
            VisionConstants.aprilTagFieldLayout,
            MULTI_TAG_PNP_ON_COPROCESSOR,
            leftCam,
            VisionConstants.leftCamToRobot);

    // Right Side Camera
    rightCam = new PhotonCamera("right");
    rightPhotonPoseEstimator =
        new PhotonPoseEstimator(
            VisionConstants.aprilTagFieldLayout,
            MULTI_TAG_PNP_ON_COPROCESSOR,
            rightCam,
            VisionConstants.rightCamToRobot);
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    getEstimatedPoseUpdates();
    inputs.visionPoses = poseArray;
    inputs.timestamps = timestampArray;
    inputs.visionStdDevs = visionStdArray;
    inputs.latency = latencyArray;
    count += 1;
    if (count % 500 == 0) {
      leftCam.takeOutputSnapshot();
      rightCam.takeOutputSnapshot();
    }
  }

  public void getEstimatedPoseUpdates() {
    Matrix<N3, N1> infiniteStdevs =
        VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

    Optional<EstimatedRobotPose> pose = leftPhotonPoseEstimator.update();
    pose.ifPresentOrElse(
        estimatedRobotPose -> {
          poseArray[0] = estimatedRobotPose.estimatedPose;
          timestampArray[0] = estimatedRobotPose.timestampSeconds;
          Matrix<N3, N1> stdDevs = getEstimationStdDevs(estimatedRobotPose);
          arraycopy(stdDevs.getData(), 0, visionStdArray, 0, 3);
          latencyArray[0] = leftCam.getLatestResult().getLatencyMillis() / 1.0e3;
        },
        () -> {
          poseArray[0] = new Pose3d();
          timestampArray[0] = 0.0;
          latencyArray[0] = 0.0;
        });
    pose = rightPhotonPoseEstimator.update();
    pose.ifPresentOrElse(
        estimatedRobotPose -> {
          poseArray[1] = estimatedRobotPose.estimatedPose;
          timestampArray[1] = estimatedRobotPose.timestampSeconds;
          Matrix<N3, N1> stdDevs = getEstimationStdDevs(estimatedRobotPose);
          arraycopy(stdDevs.getData(), 0, visionStdArray, 3, 3);
          latencyArray[1] = leftCam.getLatestResult().getLatencyMillis() / 1.0e3;
        },
        () -> {
          poseArray[1] = new Pose3d();
          timestampArray[1] = 0.0;
          latencyArray[1] = 0.0;
        });
  }

  public Matrix<N3, N1> getEstimationStdDevs(EstimatedRobotPose estimatedPose) {
    var estStdDevs = VisionConstants.singleTagStdDev;
    var targets = estimatedPose.targetsUsed;
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose
              .get()
              .toPose2d()
              .minus(estimatedPose.estimatedPose.toPose2d())
              .getTranslation()
              .getNorm();
    }

    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;

    // Decrease std devs if multiple targets are visible
    if (numTags > 1 && avgDist > 5) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs = VisionConstants.multiTagStdDev;
    }
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 20));
    }

    return estStdDevs;
  }

  @Override
  public void updatePose(Pose2d pose) {}
}
