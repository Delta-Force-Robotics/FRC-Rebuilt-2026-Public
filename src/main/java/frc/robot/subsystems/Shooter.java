package frc.robot.subsystems;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.DeviceConstants;
import frc.robot.constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

/**
 * 4-inch drum shooter driven by four Kraken X60s: one leader, one follower on the same side, and
 * two followers on the opposite side that spin the other way.
 */
public class Shooter extends SubsystemBase {
  private final TalonFX leader = new TalonFX(DeviceConstants.Shooter.LEADER_ID, Constants.RIO_BUS);
  private final TalonFX follower1 =
      new TalonFX(DeviceConstants.Shooter.FOLLOWER_1_ID, Constants.RIO_BUS);
  private final TalonFX follower2 =
      new TalonFX(DeviceConstants.Shooter.FOLLOWER_2_ID, Constants.RIO_BUS);
  private final TalonFX follower3 =
      new TalonFX(DeviceConstants.Shooter.FOLLOWER_3_ID, Constants.RIO_BUS);

  private final VelocityVoltage velocityRequest =
      new VelocityVoltage(0.0).withSlot(0).withEnableFOC(true);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private double velocitySetpointRPS = 0.0;

  private final StatusSignal<AngularVelocity> velocity = leader.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Current> leaderCurrent = leader.getStatorCurrent();
  private final StatusSignal<Current> follower1Current = follower1.getStatorCurrent();
  private final StatusSignal<Current> follower2Current = follower2.getStatorCurrent();
  private final StatusSignal<Current> follower3Current = follower3.getStatorCurrent();

  // All four motors on one shared drum shaft at 1:1
  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(4), 0.003, 1.0),
          DCMotor.getKrakenX60Foc(4));

  public Shooter() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.StatorCurrentLimit = ShooterConstants.STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SUPPLY_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = ShooterConstants.SUPPLY_CURRENT_LOWER_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLowerTime = ShooterConstants.SUPPLY_CURRENT_LOWER_TIME_SECS;
    config.Slot0.kP = ShooterConstants.VELOCITY_KP;
    config.Slot0.kI = ShooterConstants.VELOCITY_KI;
    config.Slot0.kD = ShooterConstants.VELOCITY_KD;
    config.Slot0.kS = ShooterConstants.VELOCITY_KS;
    config.Slot0.kV = ShooterConstants.VELOCITY_KV;
    config.Slot0.kA = ShooterConstants.VELOCITY_KA;
    tryUntilOk(5, () -> leader.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> follower1.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> follower2.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> follower3.getConfigurator().apply(config, 0.25));

    // follower1 is on the same side as the leader, so it spins the same direction.
    // follower2 and follower3 are on the opposite side, so they spin the other way.
    follower1.setControl(
        new Follower(DeviceConstants.Shooter.LEADER_ID, MotorAlignmentValue.Aligned));
    follower2.setControl(
        new Follower(DeviceConstants.Shooter.LEADER_ID, MotorAlignmentValue.Opposed));
    follower3.setControl(
        new Follower(DeviceConstants.Shooter.LEADER_ID, MotorAlignmentValue.Opposed));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        velocity,
        appliedVolts,
        leaderCurrent,
        follower1Current,
        follower2Current,
        follower3Current);
    leader.optimizeBusUtilization();
    follower1.optimizeBusUtilization();
    follower2.optimizeBusUtilization();
    follower3.optimizeBusUtilization();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        velocity,
        appliedVolts,
        leaderCurrent,
        follower1Current,
        follower2Current,
        follower3Current);

    Logger.recordOutput(
        "Shooter/Connected",
        new boolean[] {
          leader.isConnected(),
          follower1.isConnected(),
          follower2.isConnected(),
          follower3.isConnected()
        });
    Logger.recordOutput("Shooter/VelocityRotPerSec", getVelocity());
    Logger.recordOutput("Shooter/VelocitySetpointRPS", velocitySetpointRPS);
    Logger.recordOutput("Shooter/AppliedVolts", appliedVolts.getValueAsDouble());
    Logger.recordOutput(
        "Shooter/CurrentAmps",
        new double[] {
          leaderCurrent.getValueAsDouble(),
          follower1Current.getValueAsDouble(),
          follower2Current.getValueAsDouble(),
          follower3Current.getValueAsDouble()
        });
  }

  /** Run the motors at the given voltage (open loop). */
  public void runVolts(double volts) {
    velocitySetpointRPS = 0.0;
    leader.setControl(voltageRequest.withOutput(volts));
  }

  /** Run the motors at the given velocity in rotations per second (closed loop). */
  public void runVelocity(double rotationsPerSecond) {
    velocitySetpointRPS = rotationsPerSecond;
    leader.setControl(velocityRequest.withVelocity(rotationsPerSecond));
  }

  /** Stop the shooter. */
  public void stop() {
    runVolts(0.0);
  }

  /** Returns the drum velocity in rotations per second. */
  public double getVelocity() {
    return velocity.getValueAsDouble();
  }

  public double getVelocitySetpoint() {
    return velocitySetpointRPS;
  }

  /**
   * Steps the motor simulation forward. Runs in simulation only, from a loop faster than the main
   * robot loop (see {@link frc.robot.RobotContainer}).
   */
  public void updateSimulation(double dtSeconds) {
    var simState = leader.getSimState();
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    sim.setInputVoltage(simState.getMotorVoltage());
    sim.update(dtSeconds);
    simState.setRawRotorPosition(sim.getAngularPositionRotations());
    simState.setRotorVelocity(sim.getAngularVelocityRPM() / 60.0);
  }
}
