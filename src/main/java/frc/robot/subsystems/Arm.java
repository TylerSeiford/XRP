package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private final XRPServo servo;

  @AutoLogOutput(key = "Arm/Angle")
  private double angle = 0.0;

  public Arm() {
    servo = new XRPServo(4);
  }

  public void setAngle(double angle) {
    this.angle = angle;
    servo.setAngle(angle);
  }
}
