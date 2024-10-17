package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

public interface AprilTagVisionIO {
  @AutoLog
  public static class AprilTagVisionIOInputs {
    public Pose3d[] visionPoses =
        List.of(new Pose3d(), new Pose3d(), new Pose3d()).toArray(new Pose3d[0]);
    public double[] timestamps = new double[3];
    public double[] latency = new double[3];
    public double[] visionStdDevs = new double[9];
  }

  /** Updates the set of loggable inputs. */
  public abstract void updateInputs(AprilTagVisionIOInputs inputs);

  /** Update the reference pose of the vision system. Currently only used in sim. */
  public abstract void updatePose(Pose2d pose);

  /**
   * The standard deviations of the estimated poses from vision cameras, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  abstract Matrix<N3, N1> getEstimationStdDevs(EstimatedRobotPose estimatedPose);
}
