package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.xrp.XRPRangefinder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rangefinder extends SubsystemBase {
  private final XRPRangefinder rangefinder = new XRPRangefinder();

  @AutoLogOutput(key = "Rangefinder/DistanceMeters")
  public double getDistanceMeters() {
    return rangefinder.getDistanceMeters();
  }
}
