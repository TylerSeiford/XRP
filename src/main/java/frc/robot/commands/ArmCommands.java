package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;

public class ArmCommands {
  public static Command angleCommand(Arm arm, Angle angle) {
    return Commands.runOnce(() -> arm.setAngle(angle), arm);
  }

  public static Command adjustCommand(Arm arm, Supplier<Angle> angleSupplier) {
    return Commands.runOnce(() -> arm.setAngle(arm.getAngle().minus(angleSupplier.get())), arm);
  }
}
