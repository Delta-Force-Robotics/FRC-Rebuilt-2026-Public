package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Joystick driving commands. All driving is field relative, from the driver's point of view. */
public class DriveCommands {
  public static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 7.0;
  private static final double ANGLE_KD = 0.6;
  private static final double ANGLE_MAX_VELOCITY = 10.0;
  private static final double ANGLE_MAX_ACCELERATION = 35.0;

  private DriveCommands() {}

  /**
   * Converts joystick X/Y into a linear velocity as a fraction of max speed. Applies a deadband to
   * the stick magnitude, then squares it for finer control at low speeds.
   */
  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double magnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d direction = new Rotation2d(Math.atan2(y, x));
    return new Translation2d(magnitude * magnitude, direction);
  }

  /** Field relative drive using two joysticks (one for translation, one for rotation). */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband, then square for more precise control
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
          omega = Math.copySign(omega * omega, omega);

          drive.driveFieldRelative(
              linearVelocity.getX() * Drive.MAX_LINEAR_SPEED_MPS,
              linearVelocity.getY() * Drive.MAX_LINEAR_SPEED_MPS,
              omega * Drive.MAX_ANGULAR_SPEED_RAD_PER_SEC);
        },
        drive);
  }

  /**
   * Field relative drive using a joystick for translation and a PID controller to hold a heading.
   * Useful for snapping to an angle or aiming at a target.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              drive.driveFieldRelative(
                  linearVelocity.getX() * Drive.MAX_LINEAR_SPEED_MPS,
                  linearVelocity.getY() * Drive.MAX_LINEAR_SPEED_MPS,
                  omega);
            },
            drive)
        // Reset the PID controller when the command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }
}
