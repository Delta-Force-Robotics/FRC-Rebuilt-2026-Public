package frc.robot.subsystems;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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
import frc.robot.constants.FeederConstants;
import org.littletonrobotics.junction.Logger;

/** Two Kraken X60s feeding game pieces up into the shooter. */
public class Feeder extends SubsystemBase {
  private final TalonFX leadMotor =
      new TalonFX(DeviceConstants.Feeder.LEAD_MOTOR_ID, Constants.RIO_BUS);
  private final TalonFX followerMotor =
      new TalonFX(DeviceConstants.Feeder.FOLLOWER_MOTOR_ID, Constants.RIO_BUS);

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private double targetVolts = 0.0;

  private final StatusSignal<AngularVelocity> velocity = leadMotor.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = leadMotor.getMotorVoltage();
  private final StatusSignal<Current> leadCurrent = leadMotor.getStatorCurrent();
  private final StatusSignal<Current> followerCurrent = followerMotor.getStatorCurrent();

  // Simulation model of the lead motor
  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60Foc(1), 0.001, FeederConstants.GEAR_RATIO),
          DCMotor.getKrakenX60Foc(1));

  public Feeder() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = FeederConstants.GEAR_RATIO;
    config.CurrentLimits.StatorCurrentLimit = FeederConstants.STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = FeederConstants.SUPPLY_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> leadMotor.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> followerMotor.getConfigurator().apply(config, 0.25));

    // Follower spins the same direction as the lead
    followerMotor.setControl(
        new Follower(DeviceConstants.Feeder.LEAD_MOTOR_ID, MotorAlignmentValue.Aligned));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, appliedVolts, leadCurrent, followerCurrent);
    leadMotor.optimizeBusUtilization();
    followerMotor.optimizeBusUtilization();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(velocity, appliedVolts, leadCurrent, followerCurrent);

    Logger.recordOutput("Feeder/LeadConnected", leadMotor.isConnected());
    Logger.recordOutput("Feeder/FollowerConnected", followerMotor.isConnected());
    Logger.recordOutput("Feeder/VelocityRotPerSec", velocity.getValueAsDouble());
    Logger.recordOutput("Feeder/TargetVolts", targetVolts);
    Logger.recordOutput("Feeder/AppliedVolts", appliedVolts.getValueAsDouble());
    Logger.recordOutput("Feeder/LeadCurrentAmps", leadCurrent.getValueAsDouble());
    Logger.recordOutput("Feeder/FollowerCurrentAmps", followerCurrent.getValueAsDouble());
  }

  /** Run the feeder forward at the configured voltage. */
  public void runForward() {
    setVoltage(FeederConstants.FORWARD_VOLTS);
  }

  /** Run the feeder in reverse to eject game pieces. */
  public void runReverse() {
    setVoltage(-FeederConstants.FORWARD_VOLTS);
  }

  /** Stop the feeder. */
  public void stop() {
    setVoltage(0.0);
  }

  private void setVoltage(double volts) {
    targetVolts = volts;
    leadMotor.setControl(voltageRequest.withOutput(volts));
  }

  /** Returns true while the feeder is pushing game pieces up into the shooter. */
  public boolean isFeeding() {
    return targetVolts > 0.5;
  }

  /**
   * Steps the motor simulation forward. Runs in simulation only, from a loop faster than the main
   * robot loop (see {@link frc.robot.RobotContainer}).
   */
  public void updateSimulation(double dtSeconds) {
    var simState = leadMotor.getSimState();
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    sim.setInputVoltage(simState.getMotorVoltage());
    sim.update(dtSeconds);
    simState.setRawRotorPosition(sim.getAngularPositionRotations() * FeederConstants.GEAR_RATIO);
    simState.setRotorVelocity(sim.getAngularVelocityRPM() / 60.0 * FeederConstants.GEAR_RATIO);
  }
}
