package frc.robot;

import frc.robot.commands.ArmCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.sensors.Accelerometer;
import frc.robot.sensors.Gyro;
import frc.robot.sensors.Rangefinder;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final Drive drive = new Drive();
  private final Gyro gyro = new Gyro();
  private final Odometry odometry = new Odometry(gyro::getAngleZ, drive::getLeft, drive::getRight);
  private final DriveCommands driveCommands = new DriveCommands(drive);

  private final Arm arm = new Arm();

  private final Accelerometer accelerometer = new Accelerometer();
  private final Rangefinder rangefinder = new Rangefinder();

  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(driveCommands.drive(controller::getLeftY, controller::getLeftX));
    controller.leftStick().onTrue(driveCommands.reverse());
    controller.start().onTrue(odometry.resetAngle());

    controller.a()
        .onTrue(ArmCommands.angleCommand(arm, 0))
        .onFalse(ArmCommands.angleCommand(arm, 180));

    controller.b()
        .onTrue(ArmCommands.angleCommand(arm, 90))
        .onFalse(ArmCommands.angleCommand(arm, 180));
  }
}
