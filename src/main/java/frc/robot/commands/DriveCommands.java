package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class DriveCommands {
  private final Drive drive;

  @AutoLogOutput(key = "DriveCommands/Reversed")
  private boolean reversed = true;

  public DriveCommands(Drive drive) {
    this.drive = drive;
  }

  public Command drive(Supplier<Double> xSpeed, Supplier<Double> zRotate) {
    return drive.run(
        () -> drive.arcadeDrive(xSpeed.get() * (reversed ? -1.0 : 1.0), -zRotate.get())
    );
  }

  public Command reverse() {
    return drive.runOnce(() -> reversed = !reversed);
  }
}
