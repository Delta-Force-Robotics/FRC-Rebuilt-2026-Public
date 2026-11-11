package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.sim.MapleSimSwerveDrivetrain;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;

/**
 * Swerve drivetrain built on CTRE's Phoenix 6 swerve API. CTRE runs odometry on its own
 * high-frequency thread and owns the pose estimator. This class adds PathPlanner, vision
 * measurements, logging, and maple-sim physics in simulation.
 */
public class Drive extends TunerSwerveDrivetrain implements Subsystem {
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));
  public static final double MAX_LINEAR_SPEED_MPS =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC =
      MAX_LINEAR_SPEED_MPS / DRIVE_BASE_RADIUS;

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 58.0; // Measured
  private static final double ROBOT_MOI = 6.0; // Estimated kg*m^2
  private static final double WHEEL_COF = 0.9;

  // Simulation
  private static final double SIM_LOOP_PERIOD_SECONDS = 0.005;
  private static final Pose2d SIM_STARTING_POSE = new Pose2d(3, 3, Rotation2d.kZero);

  // Field-centric driving is relative to the driver station wall, so forward flips on red.
  private static final Rotation2d BLUE_PERSPECTIVE = Rotation2d.kZero;
  private static final Rotation2d RED_PERSPECTIVE = Rotation2d.k180deg;
  private boolean hasAppliedOperatorPerspective = false;

  private final SwerveRequest.FieldCentric fieldCentricRequest =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.Position)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
  private final SwerveRequest.ApplyRobotSpeeds robotSpeedsRequest =
      new SwerveRequest.ApplyRobotSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.Position);

  private final Field2d field = new Field2d();

  // Only created in simulation
  private MapleSimSwerveDrivetrain mapleSim = null;
  private Notifier simNotifier = null;

  public Drive(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
    register();

    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
    SmartDashboard.putData("Field", field);
  }

  private void configureAutoBuilder() {
    RobotConfig config =
        new RobotConfig(
            ROBOT_MASS_KG,
            ROBOT_MOI,
            new ModuleConfig(
                TunerConstants.FrontLeft.WheelRadius,
                MAX_LINEAR_SPEED_MPS,
                WHEEL_COF,
                DCMotor.getKrakenX60Foc(1)
                    .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                TunerConstants.FrontLeft.SlipCurrent,
                1),
            getModuleLocations());

    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getChassisSpeeds,
        (speeds, feedforwards) -> runVelocity(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(7.0, 0.0, 0.0), new PIDConstants(7.0, 0.0, 0.3)),
        config,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);

    PathPlannerLogging.setLogActivePathCallback(
        path -> Logger.recordOutput("Drive/Trajectory", path.toArray(new Pose2d[0])));
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("Drive/TrajectorySetpoint", pose));
  }

  @Override
  public void periodic() {
    // Apply the operator perspective once we know our alliance (and again whenever disabled,
    // in case it changes before the match).
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              alliance -> {
                setOperatorPerspectiveForward(
                    alliance == Alliance.Red ? RED_PERSPECTIVE : BLUE_PERSPECTIVE);
                hasAppliedOperatorPerspective = true;
              });
    }

    var state = getState();
    Logger.recordOutput("Drive/Pose", state.Pose);
    Logger.recordOutput("Drive/ChassisSpeeds", state.Speeds);
    Logger.recordOutput("Drive/OdometryPeriodSeconds", state.OdometryPeriod);
    if (state.ModuleStates != null) {
      Logger.recordOutput("Drive/ModuleStates", state.ModuleStates);
      Logger.recordOutput("Drive/ModuleTargets", state.ModuleTargets);
    }
    field.setRobotPose(state.Pose);
  }

  /**
   * Drives relative to the driver station wall.
   *
   * @param vx forward speed away from the driver station (m/s)
   * @param vy speed to the driver's left (m/s)
   * @param omega counter-clockwise rotation rate (rad/s)
   */
  public void driveFieldRelative(double vx, double vy, double omega) {
    setControl(fieldCentricRequest.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
  }

  /** Drives at the given robot-relative speeds. */
  public void runVelocity(ChassisSpeeds robotRelativeSpeeds) {
    setControl(robotSpeedsRequest.withSpeeds(robotRelativeSpeeds));
  }

  /** Stops all motion while keeping the wheels pointed where they are. */
  public void stop() {
    driveFieldRelative(0.0, 0.0, 0.0);
  }

  /** Returns the estimated robot pose on the field (blue-origin WPILib coordinates). */
  public Pose2d getPose() {
    return getState().Pose;
  }

  /** Returns the estimated robot heading on the field. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Returns the measured robot-relative chassis speeds. */
  public ChassisSpeeds getChassisSpeeds() {
    return getState().Speeds;
  }

  /** Resets the pose estimate. In simulation, also moves the physics robot to match. */
  @Override
  public void resetPose(Pose2d pose) {
    super.resetPose(pose);
    if (mapleSim != null) {
      synchronized (SimulatedArena.getInstance()) {
        mapleSim.getDriveSimulation().setSimulationWorldPose(pose);
      }
    }
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param timestampSeconds capture time as an FPGA timestamp ({@code Timer.getFPGATimestamp()})
   */
  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    super.addVisionMeasurement(
        visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), stdDevs);
  }

  /**
   * Returns the estimated pose at a past time, if it is still in the pose buffer.
   *
   * @param timestampSeconds time as an FPGA timestamp ({@code Timer.getFPGATimestamp()})
   */
  @Override
  public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
    return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
  }

  /** Returns the maple-sim drivetrain in simulation, or null on the real robot. */
  public SwerveDriveSimulation getDriveSimulation() {
    return mapleSim != null ? mapleSim.getDriveSimulation() : null;
  }

  private void startSimThread() {
    // Use the 2026 REBUILT field (must happen before anything else touches the arena)
    SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));

    mapleSim =
        new MapleSimSwerveDrivetrain(
            Seconds.of(SIM_LOOP_PERIOD_SECONDS),
            Kilograms.of(ROBOT_MASS_KG),
            Meters.of(0.8),
            Meters.of(0.8),
            DCMotor.getKrakenX60Foc(1),
            DCMotor.getKrakenX60Foc(1),
            WHEEL_COF,
            0.03, // Steer inertia for MK4i modules at the mechanism level (kg*m^2)
            getModuleLocations(),
            SIM_STARTING_POSE,
            getPigeon2(),
            getModules(),
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);
    resetPose(SIM_STARTING_POSE);

    // Run the physics faster than the main loop so the motor PID loops behave like on the robot
    simNotifier = new Notifier(mapleSim::update);
    simNotifier.setName("MapleSim");
    simNotifier.startPeriodic(SIM_LOOP_PERIOD_SECONDS);
  }
}
