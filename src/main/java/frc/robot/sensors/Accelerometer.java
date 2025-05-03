package frc.robot.sensors;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;

public class Accelerometer {
  private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  @AutoLogOutput(key = "Accelerometer/X")
  public LinearAcceleration getAccelX() {
    return Units.Gs.of(accelerometer.getX());
  }

  @AutoLogOutput(key = "Accelerometer/Y")
  public LinearAcceleration getAccelY() {
    return Units.Gs.of(accelerometer.getY());
  }

  @AutoLogOutput(key = "Accelerometer/Z")
  public LinearAcceleration getAccelZ() {
    return Units.Gs.of(accelerometer.getZ());
  }
}
