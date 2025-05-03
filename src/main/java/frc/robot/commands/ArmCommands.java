package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;

public class ArmCommands {
  public static Command angleCommand(Arm arm, double angle) {
    return Commands.runOnce(() -> arm.setAngle(angle), arm);
  }
}
