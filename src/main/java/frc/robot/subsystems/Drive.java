package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  private static final class Constants {
    public static final double GEAR_RATIO = (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
    public static final int COUNTS_PER_MOTOR_ROTATION = 12;
    public static final double WHEEL_DIAMETER = 60.0 / 1000;
  }

  private final XRPMotor leftMotor = new XRPMotor(0);
  private final XRPMotor rightMotor = new XRPMotor(1);

  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);

  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMotor::set, rightMotor::set);

  public Drive() {
    rightMotor.setInverted(true);

    leftEncoder.setDistancePerPulse(
        Math.PI * Constants.WHEEL_DIAMETER / Constants.COUNTS_PER_MOTOR_ROTATION / Constants.GEAR_RATIO);
    rightEncoder.setDistancePerPulse(
        Math.PI * Constants.WHEEL_DIAMETER / Constants.COUNTS_PER_MOTOR_ROTATION / Constants.GEAR_RATIO);
    resetEncoders();
  }

  public void arcadeDrive(double xSpeed, double zRotate) {
    differentialDrive.arcadeDrive(xSpeed, zRotate);
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  @AutoLogOutput(key = "Drive/LeftDistance")
  public Distance getLeft() {
    return Units.Meters.of(leftEncoder.getDistance());
  }

  @AutoLogOutput(key = "Drive/RightDistance")
  public Distance getRight() {
    return Units.Meters.of(rightEncoder.getDistance());
  }
}
