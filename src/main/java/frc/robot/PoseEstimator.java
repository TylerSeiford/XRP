package frc.robot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimator extends SubsystemBase {
  private final DifferentialDrivePoseEstimator3d estimator;
  private final Supplier<Rotation3d> gyroAngle;
  private final Supplier<Distance> leftDistance, rightDistance, rangeFinderDistance;

  public PoseEstimator(DifferentialDriveKinematics kinematics, Supplier<Rotation3d> gyroAngle,
      Supplier<Distance> leftDistance, Supplier<Distance> rightDistance, Supplier<Distance> rangeFinderDistance) {
    this.gyroAngle = gyroAngle;
    this.leftDistance = leftDistance;
    this.rightDistance = rightDistance;
    this.rangeFinderDistance = rangeFinderDistance;
    estimator = new DifferentialDrivePoseEstimator3d(
        kinematics,
        gyroAngle.get(),
        leftDistance.get().in(Units.Meters),
        rightDistance.get().in(Units.Meters),
        Pose3d.kZero);
  }

  @Override
  public void periodic() {
    estimator.update(
        gyroAngle.get(),
        leftDistance.get().in(Units.Meters),
        rightDistance.get().in(Units.Meters));
  }

  public void setPose(Pose3d pose) {
    estimator.resetPosition(
        gyroAngle.get(),
        leftDistance.get().in(Units.Meters),
        rightDistance.get().in(Units.Meters),
        pose);
  }

  public void addVisionMeasurement(Pose3d pose, double timestamp, Matrix<N4, N1> stdDevs) {
    estimator.addVisionMeasurement(
        pose,
        timestamp,
        stdDevs);
  }

  @AutoLogOutput(key = "PoseEstimator/Pose")
  public Pose3d getPose() {
    return estimator.getEstimatedPosition();
  }

  @AutoLogOutput(key = "PoseEstimator/RangefinderPose")
  public Pose3d getRangefinderPose() {
    return estimator.getEstimatedPosition().transformBy(
        new Transform3d(rangeFinderDistance.get(), Units.Meters.zero(), Units.Meters.zero(), Rotation3d.kZero));
  }

  public Command resetAngle() {
    return runOnce(() -> setPose(new Pose3d(
        getPose().getTranslation(),
        new Rotation3d(
            gyroAngle.get().getX(),
            gyroAngle.get().getY(),
            0))));
  }
}
