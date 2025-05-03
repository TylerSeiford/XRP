package frc.robot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Odometry extends SubsystemBase {
  private final DifferentialDriveOdometry odometry;
  private final Supplier<Angle> gyroAngle;
  private final Supplier<Distance> leftDistance, rightDistance;

  public Odometry(Supplier<Angle> gyroAngle, Supplier<Distance> leftDistance, Supplier<Distance> rightDistance) {
    this.gyroAngle = gyroAngle;
    this.leftDistance = leftDistance;
    this.rightDistance = rightDistance;
    odometry = new DifferentialDriveOdometry(
        getGyroAngle(),
        leftDistance.get(),
        rightDistance.get());
  }

  private Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees(gyroAngle.get().in(Units.Degrees));
  }

  @Override
  public void periodic() {
    odometry.update(
        getGyroAngle(),
        leftDistance.get().in(Units.Meters),
        rightDistance.get().in(Units.Meters));
  }

  public void setPose(Pose2d pose) {
    odometry.resetPosition(
        getGyroAngle(),
        leftDistance.get().in(Units.Meters),
        rightDistance.get().in(Units.Meters),
        pose);
  }

  @AutoLogOutput(key = "Odometry/Pose")
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public Command resetAngle() {
    return runOnce(() -> {
      odometry.resetPosition(
          getGyroAngle(),
          leftDistance.get().in(Units.Meters),
          rightDistance.get().in(Units.Meters),
          new Pose2d(getPose().getX(), getPose().getY(), Rotation2d.kZero));
    });
  }
}
