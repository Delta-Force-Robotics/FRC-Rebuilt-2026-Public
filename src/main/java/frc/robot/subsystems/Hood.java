package frc.robot.subsystems;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.DeviceConstants;
import frc.robot.constants.HoodConstants;
import org.littletonrobotics.junction.Logger;

/** Hood that sets the shot angle, driven by a TalonFX through a 3:1 sector gear. */
public class Hood extends SubsystemBase {
  private static final String MANUAL_ANGLE_KEY = "Hood/Tuning/ManualAngle";

  private final TalonFX motor = new TalonFX(DeviceConstants.Hood.MOTOR_ID, Constants.RIO_BUS);

  private final PositionVoltage positionRequest = new PositionVoltage(0.0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private double targetPositionRotations = 0.0;

  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = motor.getMotorVoltage();
  private final StatusSignal<Current> current = motor.getStatorCurrent();

  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX44(1), 0.001, HoodConstants.TOTAL_GEAR_RATIO),
          DCMotor.getKrakenX44(1));

  public Hood() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Internal rotor encoder with a 3:1 reduction, so position reads in hood rotations
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = HoodConstants.TOTAL_GEAR_RATIO;
    config.Slot0.kP = HoodConstants.KP;
    config.Slot0.kI = HoodConstants.KI;
    config.Slot0.kD = HoodConstants.KD;
    config.Slot0.kS = HoodConstants.KS;
    config.Slot0.kV = HoodConstants.KV;
    config.CurrentLimits.StatorCurrentLimit = HoodConstants.STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = HoodConstants.SUPPLY_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    // Soft limits stop the motor before it reaches the physical end stops
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HoodConstants.MAX_ANGLE_ROTATIONS;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HoodConstants.MIN_ANGLE_ROTATIONS;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    // Zero the encoder assuming the hood starts at the stowed position.
    // If it is not stowed on startup, remove this line and add a homing routine.
    tryUntilOk(5, () -> motor.setPosition(0.0, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
    motor.optimizeBusUtilization();

    // Manual angle for LUT tuning, editable from the dashboard
    SmartDashboard.putNumber(MANUAL_ANGLE_KEY, 0.25);

    // The hood motor is a Kraken X44. The simulated TalonFX assumes an X60 unless told otherwise.
    if (RobotBase.isSimulation()) {
      motor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX44);
    }
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);

    Logger.recordOutput("Hood/Connected", motor.isConnected());
    Logger.recordOutput("Hood/PositionRotations", getPosition());
    Logger.recordOutput("Hood/GoalPositionRotations", targetPositionRotations);
    Logger.recordOutput("Hood/AtGoal", atGoal());
    Logger.recordOutput("Hood/VelocityRotPerSec", velocity.getValueAsDouble());
    Logger.recordOutput("Hood/AppliedVolts", appliedVolts.getValueAsDouble());
    Logger.recordOutput("Hood/CurrentAmps", current.getValueAsDouble());
  }

  /** Move the hood to the given angle in mechanism rotations. */
  public void setAngle(double rotations) {
    targetPositionRotations = rotations;
    motor.setControl(positionRequest.withPosition(rotations));
  }

  /** Stop the hood motor (holds position via brake mode). */
  public void stop() {
    motor.setControl(voltageRequest.withOutput(0.0));
  }

  /** Move the hood to the dashboard "Hood/Tuning/ManualAngle" value (for LUT tuning). */
  public void setManualAngle() {
    setAngle(SmartDashboard.getNumber(MANUAL_ANGLE_KEY, 0.25));
  }

  /** Return the hood to the home position (0 rotations) to protect it from damage. */
  public void retract() {
    setAngle(0.0);
  }

  /** Returns the hood position in mechanism rotations. */
  public double getPosition() {
    return position.getValueAsDouble();
  }

  public boolean atGoal() {
    return Math.abs(getPosition() - targetPositionRotations)
        < HoodConstants.AT_GOAL_TOLERANCE_ROTATIONS;
  }

  @Override
  public void simulationPeriodic() {
    var simState = motor.getSimState();
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    sim.setInputVoltage(simState.getMotorVoltage());
    sim.update(0.02);
    simState.setRawRotorPosition(
        sim.getAngularPositionRotations() * HoodConstants.TOTAL_GEAR_RATIO);
    simState.setRotorVelocity(sim.getAngularVelocityRPM() / 60.0 * HoodConstants.TOTAL_GEAR_RATIO);
  }
}
