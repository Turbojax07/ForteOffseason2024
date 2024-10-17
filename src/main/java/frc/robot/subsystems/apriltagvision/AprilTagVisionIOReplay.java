package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.EstimatedRobotPose;

public class AprilTagVisionIOReplay implements AprilTagVisionIO {

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {}

  @Override
  public void updatePose(Pose2d pose) {}

  @Override
  public Matrix<N3, N1> getEstimationStdDevs(EstimatedRobotPose estimatedPose) {
    return null;
  }
}
