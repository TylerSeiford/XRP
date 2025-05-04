package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private static final Angle MINIMUM_ANGLE = Units.Degrees.of(0);
  private static final Angle MAXIMUM_ANGLE = Units.Degrees.of(180);

  private final XRPServo servo;

  @AutoLogOutput(key = "Arm/Angle")
  private Angle angle = Units.Degrees.zero();

  public Arm() {
    servo = new XRPServo(4);
  }

  public Angle getAngle() {
    return angle;
  }

  public void setAngle(Angle angle) {
    if (angle.gt(MAXIMUM_ANGLE)) {
      angle = MAXIMUM_ANGLE;
    } else if (angle.lt(MINIMUM_ANGLE)) {
      angle = MINIMUM_ANGLE;
    }
    this.angle = angle;
    servo.setAngle(angle.in(Units.Degrees));
  }
}
