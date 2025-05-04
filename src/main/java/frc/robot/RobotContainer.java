package frc.robot;

import frc.robot.commands.ArmCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.sensors.Accelerometer;
import frc.robot.sensors.Gyro;
import frc.robot.sensors.Rangefinder;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final Accelerometer accelerometer = new Accelerometer();
  private final Gyro gyro = new Gyro();
  private final Rangefinder rangefinder = new Rangefinder();

  private final Drive drive = new Drive();
  private final PoseEstimator poseEstimator = new PoseEstimator(Drive.KINEMATICS, gyro::getRotation3d,
      drive::getLeftDistance, drive::getRightDistance, rangefinder::getDistance);
  private final DriveCommands driveCommands = new DriveCommands(drive);

  private final Arm arm = new Arm();


  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(driveCommands.drive(controller::getLeftY, controller::getLeftX));
    controller.leftStick().onTrue(driveCommands.reverse());
    controller.start().onTrue(poseEstimator.resetAngle());

    arm.setDefaultCommand(ArmCommands.adjustCommand(arm, () -> Units.Degrees.of(controller.getRightY() * 10)));
    controller.rightStick().onTrue(ArmCommands.angleCommand(arm, Units.Degrees.of(180)));
    controller.a().onTrue(ArmCommands.angleCommand(arm, Units.Degrees.of(0)));
    controller.b().onTrue(ArmCommands.angleCommand(arm, Units.Degrees.of(90)));
  }
}
