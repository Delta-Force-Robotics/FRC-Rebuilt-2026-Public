package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.StaticShootCommand;
import frc.robot.constants.IntakeConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.sim.FuelSim;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RollerFloor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  // Subsystems
  private final Drive drive = TunerConstants.createDrivetrain();
  private final Intake intake = new Intake();
  private final RollerFloor rollerFloor = new RollerFloor();
  private final Feeder feeder = new Feeder();
  private final Shooter shooter = new Shooter();
  private final Hood hood = new Hood();
  private final Vision vision = new Vision(drive);

  // Game piece simulation (null on the real robot)
  private final FuelSim fuelSim;

  // Controller
  private final CommandPS5Controller controller = new CommandPS5Controller(0);

  // Dashboard auto selector
  private final SendableChooser<Command> autoChooser;

  // How far the hopper has extended (0.0 = retracted, 1.0 = fully out), for visualization.
  // This only increases, so the hopper stays extended after the first deployment.
  private double hopperExtendProgress = 0.0;

  public RobotContainer() {
    fuelSim =
        RobotBase.isSimulation()
            ? new FuelSim(drive.getDriveSimulation(), intake, feeder, shooter)
            : null;

    // PathPlanner named commands
    NamedCommands.registerCommand("ShootCommand1s", autoShoot().withTimeout(1.0));
    NamedCommands.registerCommand("ShootCommand2s", autoShoot().withTimeout(2.0));
    NamedCommands.registerCommand("ShootCommand3.5s", autoShoot().withTimeout(3.5));
    NamedCommands.registerCommand("ShootCommand5s", autoShoot().withTimeout(5.0));
    NamedCommands.registerCommand(
        "NoLockShootCommand3s",
        new ShootCommand(drive, shooter, hood, feeder, rollerFloor, intake, true, false)
            .withTimeout(3.0));

    new EventTrigger("DeployIntakeTrigger").onTrue(Commands.runOnce(intake::deploy));
    new EventTrigger("PartialStowIntakeTrigger").onTrue(Commands.runOnce(intake::partialStow));

    autoChooser = AutoBuilder.buildAutoChooser("None");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureButtonBindings();
  }

  /** Auto-mode shoot: aims at the hub but lets PathPlanner keep control of the drivetrain. */
  private Command autoShoot() {
    return new ShootCommand(drive, shooter, hood, feeder, rollerFloor, intake, true);
  }

  private void configureButtonBindings() {
    // Default command: normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX() * 0.8));

    // R1: lock heading to whichever of 0 or 180 degrees is closer (parallel to the field walls),
    // while still driving with the left stick
    controller
        .R1()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> {
                  double current = drive.getRotation().getRadians();
                  double distTo0 = Math.abs(MathUtil.angleModulus(current));
                  double distTo180 = Math.abs(MathUtil.angleModulus(current - Math.PI));
                  return distTo0 <= distTo180 ? Rotation2d.kZero : Rotation2d.k180deg;
                }));

    // Cross: deploy the intake and start the roller
    controller.cross().onTrue(Commands.runOnce(intake::deploy, intake));
    // Triangle: stop the roller and partially stow the intake
    controller.triangle().onTrue(Commands.runOnce(intake::partialStow, intake));

    // Square / circle: static 2 m and 3.5 m shots with no heading control
    controller
        .square()
        .whileTrue(new StaticShootCommand(shooter, hood, feeder, rollerFloor, intake, 2.0));
    controller
        .circle()
        .whileTrue(new StaticShootCommand(shooter, hood, feeder, rollerFloor, intake, 3.5));

    // D-pad down: reverse the intake roller, feeder, and roller floor to clear jams
    controller
        .povDown()
        .whileTrue(
            Commands.run(
                () -> {
                  intake.eject();
                  feeder.runReverse();
                  rollerFloor.runReverse();
                },
                intake,
                feeder,
                rollerFloor))
        .onFalse(
            Commands.runOnce(
                () -> {
                  feeder.stop();
                  rollerFloor.stop();
                },
                feeder,
                rollerFloor));

    // L1: aim at the hub (or a pass zone) and shoot. The left stick still moves the robot.
    controller
        .L1()
        .whileTrue(
            new ShootCommand(
                drive,
                shooter,
                hood,
                feeder,
                rollerFloor,
                intake,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX()));

    // Options: reset the heading so the robot's current front is "away from the driver"
    controller
        .options()
        .onTrue(Commands.runOnce(drive::seedFieldCentric, drive).ignoringDisable(true));
  }

  /** Logs 3D component poses every loop for AdvantageScope visualization. */
  public void updateVisualization() {
    Logger.recordOutput("RobotPose", drive.getPose());

    // Component 0: intake pivot (0 rot = deployed down, 0.33 rot = stowed)
    double intakeAngleRad = -intake.getPivotPosition() * 2 * Math.PI;

    // Component 1: hopper, follows the intake out on first deploy, then stays extended
    hopperExtendProgress =
        Math.max(
            hopperExtendProgress,
            1.0 - intake.getPivotPosition() / IntakeConstants.PIVOT_STOWED_ROTATIONS);
    double hopperOffsetX = MathUtil.clamp(hopperExtendProgress * 0.29, 0.0, 0.29);

    // Component 2: hood (0 rot = stowed, 1.4 rot = 32 degrees physical angle)
    double hoodAngleRad = -hood.getPosition() * (32.0 / 1.4) * Math.PI / 180.0;

    Logger.recordOutput(
        "ZeroedComponentPoses", new Pose3d[] {new Pose3d(), new Pose3d(), new Pose3d()});
    Logger.recordOutput(
        "FinalComponentPoses",
        new Pose3d[] {
          new Pose3d(0.281, 0.0, 0.195, new Rotation3d(0.0, intakeAngleRad, 0.0)),
          new Pose3d(0.159 + hopperOffsetX, 0.0, 0.395, new Rotation3d()),
          new Pose3d(-0.283, 0.0, 0.485, new Rotation3d(0.0, hoodAngleRad, 0.0))
        });
  }

  /** Called every loop in simulation to update the game piece simulation. */
  public void updateSimulation() {
    if (fuelSim != null) fuelSim.update();
  }

  /** Puts the simulated fuel back in its starting positions. Does nothing on the real robot. */
  public void resetSimulationField() {
    if (fuelSim != null) fuelSim.resetField();
  }

  /** Called at the start of auto to tell vision the pose has already been set by PathPlanner. */
  public void notifyAutoStart() {
    vision.markSeeded();
  }

  /** Returns the autonomous command selected on the dashboard. */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
