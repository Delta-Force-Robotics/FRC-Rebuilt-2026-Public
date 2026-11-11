package frc.robot.subsystems;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
import frc.robot.constants.RollerFloorConstants;
import org.littletonrobotics.junction.Logger;

/** Powered floor of the hopper that moves game pieces toward the feeder. */
public class RollerFloor extends SubsystemBase {
  private final TalonFX motor =
      new TalonFX(DeviceConstants.RollerFloor.MOTOR_ID, Constants.CANIVORE_BUS);

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private double targetVolts = 0.0;

  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = motor.getMotorVoltage();
  private final StatusSignal<Current> current = motor.getStatorCurrent();

  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60Foc(1), 0.001, RollerFloorConstants.GEAR_RATIO),
          DCMotor.getKrakenX60Foc(1));

  public RollerFloor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = RollerFloorConstants.GEAR_RATIO;
    config.CurrentLimits.StatorCurrentLimit = RollerFloorConstants.STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = RollerFloorConstants.SUPPLY_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, appliedVolts, current);
    motor.optimizeBusUtilization();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(velocity, appliedVolts, current);

    Logger.recordOutput("RollerFloor/Connected", motor.isConnected());
    Logger.recordOutput("RollerFloor/VelocityRotPerSec", velocity.getValueAsDouble());
    Logger.recordOutput("RollerFloor/TargetVolts", targetVolts);
    Logger.recordOutput("RollerFloor/AppliedVolts", appliedVolts.getValueAsDouble());
    Logger.recordOutput("RollerFloor/CurrentAmps", current.getValueAsDouble());
  }

  /** Run the roller floor forward at the configured voltage. */
  public void runForward() {
    setVoltage(RollerFloorConstants.FORWARD_VOLTS);
  }

  /** Run the roller floor in reverse to eject game pieces. */
  public void runReverse() {
    setVoltage(-RollerFloorConstants.FORWARD_VOLTS);
  }

  /** Stop the roller floor. */
  public void stop() {
    setVoltage(0.0);
  }

  private void setVoltage(double volts) {
    targetVolts = volts;
    motor.setControl(voltageRequest.withOutput(volts));
  }

  /**
   * Steps the motor simulation forward. Runs in simulation only, from a loop faster than the main
   * robot loop (see {@link frc.robot.RobotContainer}).
   */
  public void updateSimulation(double dtSeconds) {
    var simState = motor.getSimState();
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    sim.setInputVoltage(simState.getMotorVoltage());
    sim.update(dtSeconds);
    simState.setRawRotorPosition(
        sim.getAngularPositionRotations() * RollerFloorConstants.GEAR_RATIO);
    simState.setRotorVelocity(sim.getAngularVelocityRPM() / 60.0 * RollerFloorConstants.GEAR_RATIO);
  }
}
