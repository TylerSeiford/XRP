package frc.robot.sensors;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.xrp.XRPRangefinder;

public class Rangefinder {
  private final XRPRangefinder rangefinder = new XRPRangefinder();

  @AutoLogOutput(key = "Rangefinder/Distance")
  public Distance getDistance() {
    return Units.Meters.of(rangefinder.getDistanceMeters());
  }
}
