package frc.robot.sensors;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.xrp.XRPGyro;

public class Gyro {
  private final XRPGyro gyro = new XRPGyro();

  @AutoLogOutput(key = "Gyro/AngleX")
  public Angle getAngleX() {
    return Units.Degrees.of(gyro.getAngleX());
  }

  @AutoLogOutput(key = "Gyro/AngleY")
  public Angle getAngleY() {
    return Units.Degrees.of(gyro.getAngleY());
  }

  @AutoLogOutput(key = "Gyro/AngleZ")
  public Angle getAngleZ() {
    return Units.Degrees.of(gyro.getAngleZ());
  }

  @AutoLogOutput(key = "Gyro/Rotation2d")
  public Rotation2d getRotation2d() {
    return new Rotation2d(getAngleZ());
  }

  @AutoLogOutput(key = "Gyro/Rotation3d")
  public Rotation3d getRotation3d() {
    return new Rotation3d(getAngleY(), getAngleX().unaryMinus(), getAngleZ());
  }

  @AutoLogOutput(key = "Gyro/RateX")
  public AngularVelocity getRateX() {
    return Units.DegreesPerSecond.of(gyro.getRateX());
  }

  @AutoLogOutput(key = "Gyro/RateY")
  public AngularVelocity getRateY() {
    return Units.DegreesPerSecond.of(gyro.getRateY());
  }

  @AutoLogOutput(key = "Gyro/RateZ")
  public AngularVelocity getRateZ() {
    return Units.DegreesPerSecond.of(gyro.getRateZ());
  }

  public void resetGyro() {
    gyro.reset();
  }   
}
