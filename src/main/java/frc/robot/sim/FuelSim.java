package frc.robot.sim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import java.util.concurrent.ThreadLocalRandom;
import java.util.concurrent.atomic.AtomicReferenceArray;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

/**
 * Simulates fuel moving through the robot with maple-sim. The intake picks fuel up off the field
 * while its roller spins, and the shooter launches it toward the hub while the feeder is pushing
 * fuel up and the drum is spinning. Only used in simulation.
 *
 * <p>The physics world is stepped on a separate Notifier thread (see {@link
 * MapleSimSwerveDrivetrain}), so every touch of the arena here holds the arena's lock.
 */
public class FuelSim {
  private static final int MAX_ROBOT_FUEL = 50;

  // Shooter geometry and timing
  /** 4-inch drum radius in meters, used to convert RPS to exit speed. */
  private static final double DRUM_RADIUS_METERS = 0.0508;
  /** Minimum launch speed in m/s to count as a scoring attempt. */
  private static final double MIN_LAUNCH_SPEED_MPS = 3.0;
  /** Minimum shooter speed in RPS before fuel can leave the shooter. */
  private static final double MIN_SHOOTER_RPS = 5.0;
  /** Time between volleys in seconds. */
  private static final double SHOT_COOLDOWN_SECONDS = 0.15;
  /** Height of the shooter exit above the floor in meters. */
  private static final double LAUNCH_HEIGHT_METERS = 0.52;
  /** How far the shooter is from the robot center along the robot X axis in meters. */
  private static final double SHOOTER_OFFSET_X = 0.2;
  /** Y offsets for the 4 barrel positions across the shooter width. */
  private static final double[] SHOOTER_Y_OFFSETS = {-0.23, -0.077, 0.077, 0.23};
  /** Min and max fuel fired per volley (at most one per barrel). */
  private static final int VOLLEY_MIN = 1;

  private static final int VOLLEY_MAX = 4;

  // Hub position and opening height on the field in meters
  private static final double HUB_X = 4.5974;
  private static final double HUB_Y = 4.0345;
  private static final double HUB_Z = 1.85;
  private static final double GRAVITY = 9.81;
  /** Launch angle used when no trajectory reaches the hub (45 degrees gives the best range). */
  private static final double FALLBACK_PITCH_DEGREES = 45.0;

  private final AbstractDriveTrainSimulation driveSim;
  private final Intake intake;
  private final Feeder feeder;
  private final Shooter shooter;
  private final IntakeSimulation intakeSim;

  /** Fuel held anywhere inside the robot (from intake to hopper). */
  private int robotFuelCount = 0;

  private double shotCooldownTimer = SHOT_COOLDOWN_SECONDS; // start ready

  /** Latest trajectory of each barrel's shot, for AdvantageScope. */
  private final AtomicReferenceArray<Pose3d[]> trajectories =
      new AtomicReferenceArray<>(SHOOTER_Y_OFFSETS.length);

  public FuelSim(
      AbstractDriveTrainSimulation driveSim, Intake intake, Feeder feeder, Shooter shooter) {
    this.driveSim = driveSim;
    this.intake = intake;
    this.feeder = feeder;
    this.shooter = shooter;

    synchronized (SimulatedArena.getInstance()) {
      intakeSim =
          IntakeSimulation.OverTheBumperIntake(
              "Fuel",
              driveSim,
              Meters.of(0.64), // intake width
              Meters.of(0.23), // extension length beyond bumper
              IntakeSide.FRONT,
              5); // collect up to 5 fuel per pass from a pile
      intakeSim.register();
    }
  }

  /** Call once per robot loop in simulation. */
  public void update() {
    SimulatedArena arena = SimulatedArena.getInstance();
    synchronized (arena) {
      updateIntake();
      updateShooter();

      Pose3d[] fuelPoses = arena.getGamePiecesArrayByType("Fuel");
      Logger.recordOutput("FuelSim/FieldFuelPoses", fuelPoses);
      Logger.recordOutput("FuelSim/FieldFuelCount", fuelPoses.length);
      Logger.recordOutput("FuelSim/FuelInAir", arena.gamePieceLaunched().size());
      Logger.recordOutput("FuelSim/RobotFuelCount", robotFuelCount);
      Logger.recordOutput("FuelSim/IntakeBufferCount", intakeSim.getGamePiecesAmount());
      Logger.recordOutput("FuelSim/RobotPose", driveSim.getSimulatedDriveTrainPose());
    }
    for (int i = 0; i < trajectories.length(); i++) {
      Pose3d[] trajectory = trajectories.get(i);
      Logger.recordOutput(
          "FuelSim/Trajectory/" + i, trajectory != null ? trajectory : new Pose3d[0]);
    }
  }

  /** Puts all the fuel back in its starting positions, for example before auto. */
  public void resetField() {
    SimulatedArena arena = SimulatedArena.getInstance();
    synchronized (arena) {
      arena.resetFieldForAuto();
    }
  }

  private void updateIntake() {
    // Only collect fuel while the roller spins inward. This also prevents fuel bouncing out of
    // the hub from being picked back up while shooting.
    if (intake.isRollerIntaking()) {
      intakeSim.startIntake();
      while (robotFuelCount < MAX_ROBOT_FUEL && intakeSim.getGamePiecesAmount() > 0) {
        intakeSim.obtainGamePieceFromIntake();
        robotFuelCount++;
      }
    } else {
      intakeSim.stopIntake();
    }
  }

  private void updateShooter() {
    shotCooldownTimer += 0.02;
    double shooterRps = shooter.getVelocity();
    if (feeder.isFeeding()
        && shooterRps >= MIN_SHOOTER_RPS
        && shotCooldownTimer >= SHOT_COOLDOWN_SECONDS
        && robotFuelCount > 0) {
      int volley = ThreadLocalRandom.current().nextInt(VOLLEY_MIN, VOLLEY_MAX + 1);
      int toFire = Math.min(volley, robotFuelCount);
      robotFuelCount -= toFire;
      launchVolley(shooterRps, toFire);
      shotCooldownTimer = 0.0;
    }
  }

  /**
   * Launches fuel toward the hub, one piece per barrel. The hub automatically ejects scored fuel
   * back onto the field.
   */
  private void launchVolley(double shooterRps, int count) {
    Pose2d robotPose = driveSim.getSimulatedDriveTrainPose();
    // The shooter faces backwards
    Rotation2d shooterFacing = robotPose.getRotation().plus(Rotation2d.k180deg);
    double speed = Math.max(shooterRps * 2.0 * Math.PI * DRUM_RADIUS_METERS, MIN_LAUNCH_SPEED_MPS);
    double deltaH = HUB_Z - LAUNCH_HEIGHT_METERS;

    for (int i = 0; i < count; i++) {
      // Find each barrel's position on the field to get an accurate distance to the hub
      Translation2d barrelOffset = new Translation2d(SHOOTER_OFFSET_X, SHOOTER_Y_OFFSETS[i]);
      Translation2d barrel = robotPose.getTranslation().plus(barrelOffset.rotateBy(shooterFacing));
      double horizontalDistance = Math.hypot(barrel.getX() - HUB_X, barrel.getY() - HUB_Y);
      double pitchDegrees = Math.toDegrees(computeLowArcAngle(speed, horizontalDistance, deltaH));

      final int barrelIndex = i;
      SimulatedArena.getInstance()
          .addGamePieceProjectile(
              new RebuiltFuelOnFly(
                      robotPose.getTranslation(),
                      barrelOffset,
                      new ChassisSpeeds(),
                      shooterFacing,
                      Meters.of(LAUNCH_HEIGHT_METERS),
                      MetersPerSecond.of(speed),
                      Degrees.of(pitchDegrees))
                  // maple-sim may call this from the physics thread, so just store it here
                  .withProjectileTrajectoryDisplayCallBack(
                      trajectory ->
                          trajectories.set(barrelIndex, trajectory.toArray(new Pose3d[0]))));
    }
    Logger.recordOutput("FuelSim/LaunchSpeedMPS", speed);
  }

  /**
   * Returns the shallower launch angle in radians that reaches the hub at this speed, or 45 degrees
   * if the speed is too low to reach it.
   */
  private static double computeLowArcAngle(double speed, double horizDist, double deltaH) {
    double a = GRAVITY * horizDist * horizDist / (2.0 * speed * speed);
    double discriminant = horizDist * horizDist - 4.0 * a * (deltaH + a);
    if (discriminant < 0.0) {
      return Math.toRadians(FALLBACK_PITCH_DEGREES);
    }
    double angleLow = Math.atan((horizDist - Math.sqrt(discriminant)) / (2.0 * a));
    return angleLow > 0.0 ? angleLow : Math.toRadians(FALLBACK_PITCH_DEGREES);
  }
}
