package frc.robot.sim;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * Connects CTRE's swerve drivetrain to a maple-sim physics drivetrain. Each simulation step, maple
 * reads the voltage the simulated TalonFXs are applying, moves the robot through the physics world
 * (bumping into field elements and game pieces), then writes the resulting encoder and gyro
 * readings back into the CTRE sim devices.
 *
 * <p>Based on the CTRE integration example from the maple-sim documentation.
 */
public class MapleSimSwerveDrivetrain {
  private final Pigeon2SimState pigeonSim;
  private final SwerveDriveSimulation mapleSimDrive;

  @SafeVarargs
  public MapleSimSwerveDrivetrain(
      Time simPeriod,
      Mass robotMassWithBumpers,
      Distance bumperLengthX,
      Distance bumperWidthY,
      DCMotor driveMotorModel,
      DCMotor steerMotorModel,
      double wheelCOF,
      double steerInertiaKgM2,
      Translation2d[] moduleLocations,
      Pose2d startingPose,
      Pigeon2 pigeon,
      SwerveModule<TalonFX, TalonFX, CANcoder>[] modules,
      SwerveModuleConstants<?, ?, ?>... moduleConstants) {
    pigeonSim = pigeon.getSimState();

    var constants = moduleConstants[0];
    DriveTrainSimulationConfig config =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(robotMassWithBumpers)
            .withBumperSize(bumperLengthX, bumperWidthY)
            .withGyro(COTS.ofPigeon2())
            .withCustomModuleTranslations(moduleLocations)
            .withSwerveModule(
                new SwerveModuleSimulationConfig(
                    driveMotorModel,
                    steerMotorModel,
                    constants.DriveMotorGearRatio,
                    constants.SteerMotorGearRatio,
                    Volts.of(constants.DriveFrictionVoltage),
                    Volts.of(constants.SteerFrictionVoltage),
                    Meters.of(constants.WheelRadius),
                    KilogramSquareMeters.of(steerInertiaKgM2),
                    wheelCOF));
    mapleSimDrive = new SwerveDriveSimulation(config, startingPose);

    // Let maple-sim drive the simulated CTRE motors and encoders
    SwerveModuleSimulation[] moduleSims = mapleSimDrive.getModules();
    for (int i = 0; i < moduleSims.length; i++) {
      moduleSims[i].useDriveMotorController(new TalonFXMotorSim(modules[i].getDriveMotor()));
      moduleSims[i].useSteerMotorController(
          new TalonFXWithCANcoderMotorSim(modules[i].getSteerMotor(), modules[i].getEncoder()));
    }

    // One physics step per call, since update() is called from a fast Notifier
    SimulatedArena.overrideSimulationTimings(simPeriod, 1);
    SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);
  }

  /** Steps the whole simulated field forward and updates the simulated gyro. */
  public void update() {
    SimulatedArena arena = SimulatedArena.getInstance();
    synchronized (arena) {
      arena.simulationPeriodic();
      // Use maple's simulated gyro rather than the physics pose. The gyro only integrates
      // rotation, so teleporting the robot (resetPose) doesn't look like a sudden heading jump.
      var gyroSim = mapleSimDrive.getGyroSimulation();
      pigeonSim.setRawYaw(gyroSim.getGyroReading().getMeasure());
      pigeonSim.setAngularVelocityZ(gyroSim.getMeasuredAngularVelocity());
    }
  }

  /** Returns the maple-sim drivetrain, used for the true robot pose and the intake simulation. */
  public SwerveDriveSimulation getDriveSimulation() {
    return mapleSimDrive;
  }

  /**
   * In simulation, encoder offsets and inversions only get in the way, since maple-sim already
   * models each module in its own frame. Clears them so the simulated modules behave. Does nothing
   * on the real robot.
   */
  public static SwerveModuleConstants<?, ?, ?>[] regulateModuleConstantsForSimulation(
      SwerveModuleConstants<?, ?, ?>[] moduleConstants) {
    if (RobotBase.isReal()) return moduleConstants;
    for (var constants : moduleConstants) {
      constants
          .withEncoderOffset(0)
          .withDriveMotorInverted(false)
          .withSteerMotorInverted(false)
          .withEncoderInverted(false);
    }
    return moduleConstants;
  }

  /** Feeds a simulated TalonFX with the rotor position and velocity from maple-sim. */
  private static class TalonFXMotorSim implements SimulatedMotorController {
    private final TalonFXSimState talonSim;

    TalonFXMotorSim(TalonFX talon) {
      talonSim = talon.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      talonSim.setRawRotorPosition(encoderAngle);
      talonSim.setRotorVelocity(encoderVelocity);
      talonSim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      return talonSim.getMotorVoltageMeasure();
    }
  }

  /** Same as {@link TalonFXMotorSim}, but also feeds the module's CANcoder. */
  private static class TalonFXWithCANcoderMotorSim extends TalonFXMotorSim {
    private final CANcoderSimState encoderSim;

    TalonFXWithCANcoderMotorSim(TalonFX talon, CANcoder encoder) {
      super(talon);
      encoderSim = encoder.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      encoderSim.setRawPosition(mechanismAngle);
      encoderSim.setVelocity(mechanismVelocity);
      return super.updateControlSignal(
          mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
    }
  }
}
