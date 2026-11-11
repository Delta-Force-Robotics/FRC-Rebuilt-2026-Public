package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.ShootComConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RollerFloor;
import frc.robot.subsystems.Shooter;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Points the robot at the hub or a pass zone based on where we are on the field, then spins up the
 * shooter and sets the hood angle. Once the heading and speed are both ready, it runs the feeder
 * and slowly retracts the intake to partial stow. When released, it leaves the shooter spinning
 * slowly.
 */
public class ShootCommand extends Command {

  private enum TargetType {
    HUB,
    PASS_1,
    PASS_2
  }

  // Reuse a single Rotation2d for 180 degrees to avoid creating a new one every loop.
  private static final Rotation2d ROT_PI = new Rotation2d(Math.PI);

  private final Drive drive;
  private final Shooter shooter;
  private final Hood hood;
  private final Feeder feeder;
  private final RollerFloor rollerFloor;
  private final Intake intake;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  // In auto mode, PathPlanner controls the drive so we only read the pose, not command it.
  private final boolean autoMode;
  // When false, skips heading lock so PathPlanner's lock-to-point can control rotation instead.
  private final boolean lockHeading;

  private final ProfiledPIDController headingController =
      new ProfiledPIDController(
          ShootComConstants.HEADING_KP,
          0.0,
          ShootComConstants.HEADING_KD,
          new TrapezoidProfile.Constraints(
              ShootComConstants.HEADING_MAX_VEL_RAD_PER_S,
              ShootComConstants.HEADING_MAX_ACCEL_RAD_PER_S2));

  private TargetType targetType = TargetType.HUB;
  // Saved here so isReadyToFire() can read it without recalculating each call.
  private Rotation2d currentTargetHeading = Rotation2d.kZero;

  /**
   * Auto mode constructor. Does not require drive (so PathPlanner can still run paths in parallel),
   * but still commands drive heading when {@code lockHeading=true}. Use this for named commands
   * that run sequentially between paths where the robot needs to rotate to aim.
   */
  public ShootCommand(
      Drive drive,
      Shooter shooter,
      Hood hood,
      Feeder feeder,
      RollerFloor rollerFloor,
      Intake intake,
      boolean autoMode) {
    this(drive, shooter, hood, feeder, rollerFloor, intake, autoMode, true);
  }

  /**
   * Auto mode constructor with optional heading lock. Pass {@code lockHeading=false} to let
   * PathPlanner's lock-to-point feature control rotation instead of this command's heading PID.
   */
  public ShootCommand(
      Drive drive,
      Shooter shooter,
      Hood hood,
      Feeder feeder,
      RollerFloor rollerFloor,
      Intake intake,
      boolean autoMode,
      boolean lockHeading) {
    this.drive = drive;
    this.shooter = shooter;
    this.hood = hood;
    this.feeder = feeder;
    this.rollerFloor = rollerFloor;
    this.intake = intake;
    this.xSupplier = () -> 0.0;
    this.ySupplier = () -> 0.0;
    this.autoMode = autoMode;
    this.lockHeading = lockHeading;
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    // Drive not required here because PathPlanner controls it in auto
    addRequirements(shooter, hood, feeder, rollerFloor, intake);
  }

  /** Aims and shoots while the driver can still move with the left joystick. */
  public ShootCommand(
      Drive drive,
      Shooter shooter,
      Hood hood,
      Feeder feeder,
      RollerFloor rollerFloor,
      Intake intake,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    this.drive = drive;
    this.shooter = shooter;
    this.hood = hood;
    this.feeder = feeder;
    this.rollerFloor = rollerFloor;
    this.intake = intake;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.autoMode = false;
    this.lockHeading = true;
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive, shooter, hood, feeder, rollerFloor, intake);
  }

  @Override
  public void initialize() {
    headingController.reset(drive.getRotation().getRadians());
    updateTargetType();
    // Pre-seed shooter/hood so they start spinning before the first execute tick
    applyShooterState();
  }

  @Override
  public void execute() {
    updateTargetType();

    Pose2d pose = drive.getPose();
    Translation2d target = getTarget();

    double dx = target.getX() - pose.getX();
    double dy = target.getY() - pose.getY();
    double distance = Math.hypot(dx, dy);

    currentTargetHeading = new Rotation2d(dx, dy).plus(ROT_PI);

    // Always spin up shooter and position hood
    shooter.runVelocity(ShootComConstants.SHOOT_VELOCITY_LUT.get(distance));
    hood.setAngle(ShootComConstants.HOOD_ANGLE_LUT.get(distance));

    // Feed only when heading and shooter speed are both within tolerance.
    boolean ready = isReadyToFire();
    if (ready) {
      rollerFloor.runForward();
      feeder.runForward();
      intake.slowPartialStow();
    } else {
      rollerFloor.stop();
      feeder.stop();
    }
    Logger.recordOutput("ShootCommand/TargetType", targetType.toString());
    Logger.recordOutput("ShootCommand/DistanceMeters", distance);
    Logger.recordOutput("ShootCommand/TargetHeadingDeg", currentTargetHeading.getDegrees());
    Logger.recordOutput("ShootCommand/ReadyToFire", ready);

    if (lockHeading) {
      double omega =
          headingController.calculate(
              drive.getRotation().getRadians(), currentTargetHeading.getRadians());
      // Driver can still translate, but slower so aiming stays stable
      Translation2d linearVelocity =
          DriveCommands.getLinearVelocityFromJoysticks(
              xSupplier.getAsDouble(), ySupplier.getAsDouble());
      double maxSpeed = Drive.MAX_LINEAR_SPEED_MPS * ShootComConstants.SHOOT_DRIVE_SPEED_FACTOR;
      drive.driveFieldRelative(
          linearVelocity.getX() * maxSpeed, linearVelocity.getY() * maxSpeed, omega);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Keep the shooter spinning slowly so it gets back to speed faster next time
    shooter.runVolts(ShootComConstants.SHOOTER_IDLE_VOLTS);
    hood.retract();
    feeder.stop();
    rollerFloor.stop();
    if (!autoMode) {
      drive.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Returns true when the robot is aimed well enough and the shooter is at the right speed. The
   * feeder and roller floor only run when this is true.
   */
  private boolean isReadyToFire() {
    double shooterError = Math.abs(shooter.getVelocity() - shooter.getVelocitySetpoint());
    boolean shooterOk = shooterError <= ShootComConstants.SHOOTER_READY_RPS_TOLERANCE;
    Logger.recordOutput("ShootCommand/ShooterErrorRPS", shooterError);
    if (!lockHeading) {
      // Heading lock disabled (e.g. NoLockShootCommand): only check shooter speed
      return shooterOk;
    }
    double headingError =
        Math.abs(
            MathUtil.angleModulus(
                drive.getRotation().getRadians() - currentTargetHeading.getRadians()));
    boolean headingOk = headingError < ShootComConstants.HEADING_TOLERANCE_RADIANS;
    Logger.recordOutput("ShootCommand/HeadingErrorRad", headingError);
    return headingOk && shooterOk;
  }

  private void updateTargetType() {
    Pose2d pose = drive.getPose();
    if (isInShootZone(pose)) {
      targetType = TargetType.HUB;
    } else if (pose.getY() < ShootComConstants.PASS_ZONE_Y_THRESHOLD) {
      targetType = TargetType.PASS_1;
    } else {
      targetType = TargetType.PASS_2;
    }
  }

  /** Looks up shooter speed and hood angle from the distance tables and applies them. */
  private void applyShooterState() {
    double distance = drive.getPose().getTranslation().getDistance(getTarget());
    shooter.runVelocity(ShootComConstants.SHOOT_VELOCITY_LUT.get(distance));
    hood.setAngle(ShootComConstants.HOOD_ANGLE_LUT.get(distance));
  }

  private Translation2d getTarget() {
    return switch (targetType) {
      case HUB -> flipIfRed(ShootComConstants.HUB_BLUE);
      case PASS_1 -> flipIfRed(ShootComConstants.PASS_ZONE_1_BLUE);
      case PASS_2 -> flipIfRed(ShootComConstants.PASS_ZONE_2_BLUE);
    };
  }

  /**
   * Returns true if the robot is inside the shoot zone. The zone is defined for blue side and the X
   * boundary is flipped for red alliance.
   */
  private boolean isInShootZone(Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    double xMax =
        isRed
            ? Constants.FIELD_LENGTH_METERS - ShootComConstants.SHOOT_ZONE_X_MAX
            : ShootComConstants.SHOOT_ZONE_X_MAX;

    boolean xInZone = isRed ? x >= xMax : x <= xMax;
    boolean yInZone =
        y >= ShootComConstants.SHOOT_ZONE_Y_MIN && y <= ShootComConstants.SHOOT_ZONE_Y_MAX;
    return xInZone && yInZone;
  }

  /** Flips a blue-side target to the red side when we are on red alliance. */
  private Translation2d flipIfRed(Translation2d blueTarget) {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    return isRed
        ? new Translation2d(Constants.FIELD_LENGTH_METERS - blueTarget.getX(), blueTarget.getY())
        : blueTarget;
  }
}
