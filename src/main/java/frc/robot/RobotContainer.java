package frc.robot;

import frc.robot.commands.ArmCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Rangefinder;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final Drive drive = new Drive();
  private final DriveCommands driveCommands = new DriveCommands(drive);
  private final Arm arm = new Arm();
  private final Rangefinder rangefinder = new Rangefinder();

  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(driveCommands.drive(controller::getLeftY, controller::getLeftX));
    controller.leftStick().onTrue(driveCommands.reverse());

    controller.a()
        .onTrue(ArmCommands.angleCommand(arm, 0))
        .onFalse(ArmCommands.angleCommand(arm, 180));

    controller.b()
        .onTrue(ArmCommands.angleCommand(arm, 90))
        .onFalse(ArmCommands.angleCommand(arm, 180));
  }
}
