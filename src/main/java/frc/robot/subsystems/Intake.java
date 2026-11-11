package frc.robot.subsystems;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.DeviceConstants;
import frc.robot.constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Over-the-bumper intake: a pivoting arm (TalonFX with a fused CANcoder) and a roller driven by two
 * TalonFXs. A small state machine in {@link #periodic()} decides where the arm goes and whether the
 * roller spins.
 */
public class Intake extends SubsystemBase {

  public enum IntakeState {
    /** Pivot at stowed position, roller stopped. */
    STOWED,
    /** Pivot moving toward deployed position, roller running. */
    DEPLOYING,
    /** Pivot at deployed position, roller running to intake game pieces. */
    INTAKING,
    /** Pivot at deployed position, roller stopped. */
    DEPLOYED_IDLE,
    /** Pivot moving back to stowed position, roller running to push fuel into the robot. */
    RETRACTING,
    /** Pivot moving to partial stow position, roller stopped. */
    PARTIAL_STOWING,
    /** Pivot at partial stow position, roller stopped. */
    PARTIAL_STOWED,
    /**
     * Pivot slowly stepping toward partial stow at a controlled rate, roller running to feed fuel
     * smoothly. Used during shooting to avoid jamming fuel.
     */
    SLOW_PARTIAL_STOWING
  }

  private static final double PIVOT_TOTAL_GEAR_RATIO =
      IntakeConstants.ROTOR_TO_SENSOR_RATIO * IntakeConstants.SENSOR_TO_MECHANISM_RATIO;

  private final TalonFX pivotMotor =
      new TalonFX(DeviceConstants.Intake.PIVOT_MOTOR_ID, Constants.CANIVORE_BUS);
  private final CANcoder pivotEncoder =
      new CANcoder(DeviceConstants.Intake.PIVOT_CANCODER_ID, Constants.CANIVORE_BUS);
  private final TalonFX rollerMotor =
      new TalonFX(DeviceConstants.Intake.ROLLER_MOTOR_ID, Constants.RIO_BUS);
  private final TalonFX rollerFollowerMotor =
      new TalonFX(DeviceConstants.Intake.ROLLER_FOLLOWER_MOTOR_ID, Constants.RIO_BUS);

  private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0.0).withSlot(0);
  private final VoltageOut rollerRequest = new VoltageOut(0.0);

  private final StatusSignal<Angle> pivotPosition = pivotMotor.getPosition();
  private final StatusSignal<AngularVelocity> pivotVelocity = pivotMotor.getVelocity();
  private final StatusSignal<Voltage> pivotAppliedVolts = pivotMotor.getMotorVoltage();
  private final StatusSignal<Current> pivotCurrent = pivotMotor.getStatorCurrent();
  private final StatusSignal<AngularVelocity> rollerVelocity = rollerMotor.getVelocity();
  private final StatusSignal<Voltage> rollerAppliedVolts = rollerMotor.getMotorVoltage();
  private final StatusSignal<Current> rollerCurrent = rollerMotor.getStatorCurrent();

  private IntakeState state = IntakeState.STOWED;
  private double pivotGoalRotations = IntakeConstants.PIVOT_STOWED_ROTATIONS;
  private double rollerTargetVolts = 0.0;
  // Incrementally-stepped pivot target used during SLOW_PARTIAL_STOWING
  private double slowRetractTarget = 0.0;

  // Simulation model of the pivot arm, in pivot axle rotations. Starts stowed like the real robot.
  private final DCMotorSim pivotSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60Foc(1), 0.002, PIVOT_TOTAL_GEAR_RATIO),
          DCMotor.getKrakenX60Foc(1));

  public Intake() {
    // Absolute encoder on the pivot
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = IntakeConstants.CANCODER_OFFSET_ROTATIONS;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    tryUntilOk(5, () -> pivotEncoder.getConfigurator().apply(encoderConfig, 0.25));

    // Pivot motor, using the CANcoder fused with the rotor sensor as feedback.
    // 37.5:1 from motor to CANcoder, then 2:1 from CANcoder to pivot axle, so the
    // TalonFX reports position in pivot axle rotations.
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.Feedback.FeedbackRemoteSensorID = DeviceConstants.Intake.PIVOT_CANCODER_ID;
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    pivotConfig.Feedback.RotorToSensorRatio = IntakeConstants.ROTOR_TO_SENSOR_RATIO;
    pivotConfig.Feedback.SensorToMechanismRatio = IntakeConstants.SENSOR_TO_MECHANISM_RATIO;
    pivotConfig.Slot0.kP = IntakeConstants.PIVOT_KP;
    pivotConfig.Slot0.kI = IntakeConstants.PIVOT_KI;
    pivotConfig.Slot0.kD = IntakeConstants.PIVOT_KD;
    pivotConfig.Slot0.kS = IntakeConstants.PIVOT_KS;
    pivotConfig.Slot0.kV = IntakeConstants.PIVOT_KV;
    pivotConfig.Slot0.kA = IntakeConstants.PIVOT_KA;
    pivotConfig.Slot0.kG = IntakeConstants.PIVOT_KG;
    pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity =
        IntakeConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    pivotConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.MOTION_MAGIC_ACCELERATION;
    pivotConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.PIVOT_STATOR_CURRENT_LIMIT_AMPS;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.PIVOT_SUPPLY_CURRENT_LIMIT_AMPS;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotConfig, 0.25));

    // Roller motors, with a 1:2 reduction. The follower faces the other way.
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.Feedback.SensorToMechanismRatio = IntakeConstants.ROLLER_GEAR_RATIO;
    rollerConfig.CurrentLimits.StatorCurrentLimit =
        IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT_AMPS;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit =
        IntakeConstants.ROLLER_SUPPLY_CURRENT_LIMIT_AMPS;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(rollerConfig, 0.25));
    tryUntilOk(5, () -> rollerFollowerMotor.getConfigurator().apply(rollerConfig, 0.25));
    rollerFollowerMotor.setControl(
        new Follower(DeviceConstants.Intake.ROLLER_MOTOR_ID, MotorAlignmentValue.Opposed));

    // The pivot is on the CANivore and the roller is on the RIO bus. Phoenix only accepts signals
    // from one CAN bus per call, so each bus gets its own call.
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, pivotPosition, pivotVelocity, pivotAppliedVolts, pivotCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, rollerVelocity, rollerAppliedVolts, rollerCurrent);
    pivotMotor.optimizeBusUtilization();
    rollerMotor.optimizeBusUtilization();
    rollerFollowerMotor.optimizeBusUtilization();

    if (RobotBase.isSimulation()) {
      setUpSimulation();
    }
  }

  @Override
  public void periodic() {
    // One call per CAN bus (see constructor)
    BaseStatusSignal.refreshAll(pivotPosition, pivotVelocity, pivotAppliedVolts, pivotCurrent);
    BaseStatusSignal.refreshAll(rollerVelocity, rollerAppliedVolts, rollerCurrent);

    switch (state) {
      case STOWED -> setOutputs(IntakeConstants.PIVOT_STOWED_ROTATIONS, 0.0);
      case DEPLOYING -> {
        setOutputs(IntakeConstants.PIVOT_DEPLOYED_ROTATIONS, IntakeConstants.ROLLER_INTAKE_VOLTS);
        if (isPivotNear(IntakeConstants.PIVOT_DEPLOYED_ROTATIONS)) state = IntakeState.INTAKING;
      }
      case INTAKING -> setOutputs(
          IntakeConstants.PIVOT_DEPLOYED_ROTATIONS, IntakeConstants.ROLLER_INTAKE_VOLTS);
      case DEPLOYED_IDLE -> setOutputs(IntakeConstants.PIVOT_DEPLOYED_ROTATIONS, 0.0);
      case RETRACTING -> {
        setOutputs(IntakeConstants.PIVOT_STOWED_ROTATIONS, IntakeConstants.ROLLER_INTAKE_VOLTS);
        if (isPivotNear(IntakeConstants.PIVOT_STOWED_ROTATIONS)) state = IntakeState.STOWED;
      }
      case PARTIAL_STOWING -> {
        setOutputs(IntakeConstants.PIVOT_PARTIAL_STOW_ROTATIONS, 0.0);
        if (isPivotNear(IntakeConstants.PIVOT_PARTIAL_STOW_ROTATIONS)) {
          state = IntakeState.PARTIAL_STOWED;
        }
      }
      case PARTIAL_STOWED -> setOutputs(IntakeConstants.PIVOT_PARTIAL_STOW_ROTATIONS, 0.0);
      case SLOW_PARTIAL_STOWING -> {
        slowRetractTarget =
            Math.min(
                slowRetractTarget + IntakeConstants.SLOW_PARTIAL_STOW_RATE_ROT_PER_S * 0.02,
                IntakeConstants.PIVOT_PARTIAL_STOW_ROTATIONS);
        setOutputs(slowRetractTarget, IntakeConstants.ROLLER_INTAKE_VOLTS);
      }
    }

    Logger.recordOutput("Intake/State", state.toString());
    Logger.recordOutput("Intake/PivotConnected", pivotMotor.isConnected());
    Logger.recordOutput("Intake/PivotEncoderConnected", pivotEncoder.isConnected());
    Logger.recordOutput("Intake/PivotPositionRotations", getPivotPosition());
    Logger.recordOutput("Intake/PivotGoalRotations", pivotGoalRotations);
    Logger.recordOutput("Intake/PivotVelocityRotPerSec", pivotVelocity.getValueAsDouble());
    Logger.recordOutput("Intake/PivotAppliedVolts", pivotAppliedVolts.getValueAsDouble());
    Logger.recordOutput("Intake/PivotCurrentAmps", pivotCurrent.getValueAsDouble());
    Logger.recordOutput("Intake/RollerConnected", rollerMotor.isConnected());
    Logger.recordOutput("Intake/RollerVelocityRotPerSec", rollerVelocity.getValueAsDouble());
    Logger.recordOutput("Intake/RollerTargetVolts", rollerTargetVolts);
    Logger.recordOutput("Intake/RollerAppliedVolts", rollerAppliedVolts.getValueAsDouble());
    Logger.recordOutput("Intake/RollerCurrentAmps", rollerCurrent.getValueAsDouble());
  }

  /** Sends the pivot to a position (MotionMagic) and runs the roller at a voltage. */
  private void setOutputs(double pivotRotations, double rollerVolts) {
    pivotGoalRotations = pivotRotations;
    pivotMotor.setControl(pivotRequest.withPosition(pivotRotations));
    setRollerVoltage(rollerVolts);
  }

  private void setRollerVoltage(double volts) {
    rollerTargetVolts = volts;
    rollerMotor.setControl(rollerRequest.withOutput(volts));
  }

  /**
   * Start deploying the intake and run the roller. If already deployed idle, immediately resumes
   * the roller without re-deploying.
   */
  public void deploy() {
    if (state == IntakeState.STOWED
        || state == IntakeState.PARTIAL_STOWED
        || state == IntakeState.PARTIAL_STOWING
        || state == IntakeState.SLOW_PARTIAL_STOWING) {
      state = IntakeState.DEPLOYING;
    } else if (state == IntakeState.DEPLOYED_IDLE) {
      state = IntakeState.INTAKING;
    }
  }

  /**
   * Stop the roller while keeping the pivot at the deployed position. No-op if already stowed or
   * retracting.
   */
  public void stopRoller() {
    if (state == IntakeState.DEPLOYING || state == IntakeState.INTAKING) {
      state = IntakeState.DEPLOYED_IDLE;
    }
  }

  /** Retract the intake to fully stowed position. */
  public void retract() {
    if (state == IntakeState.DEPLOYING
        || state == IntakeState.INTAKING
        || state == IntakeState.DEPLOYED_IDLE
        || state == IntakeState.PARTIAL_STOWING
        || state == IntakeState.PARTIAL_STOWED) {
      state = IntakeState.RETRACTING;
    }
  }

  /** Move the intake to the partial stow position. Stops the roller. */
  public void partialStow() {
    state = IntakeState.PARTIAL_STOWING;
  }

  /**
   * Slowly move the intake toward the partial stow position at a controlled rate. Used during
   * shooting so fuel feeds smoothly without jamming. Starts from the current position so motion is
   * continuous regardless of where the arm is.
   */
  public void slowPartialStow() {
    if (state != IntakeState.SLOW_PARTIAL_STOWING) {
      slowRetractTarget = getPivotPosition();
      state = IntakeState.SLOW_PARTIAL_STOWING;
    }
  }

  /**
   * Run the roller in reverse to eject a game piece. Does not change the pivot state, so the state
   * machine takes the roller back over on the next loop once this stops being called.
   */
  public void eject() {
    setRollerVoltage(IntakeConstants.ROLLER_EJECT_VOLTS);
  }

  public IntakeState getState() {
    return state;
  }

  /** Returns the pivot position in pivot axle rotations (0 = deployed). */
  public double getPivotPosition() {
    return pivotPosition.getValueAsDouble();
  }

  /** Returns true while the roller is spinning inward. */
  public boolean isRollerIntaking() {
    return rollerTargetVolts > 0.5;
  }

  private boolean isPivotNear(double targetRotations) {
    return Math.abs(getPivotPosition() - targetRotations)
        < IntakeConstants.PIVOT_AT_GOAL_TOLERANCE_ROTATIONS;
  }

  // ---------------- Simulation ----------------

  private void setUpSimulation() {
    // The CANcoder reads the opposite direction of the motor, and its magnet offset is applied on
    // top of the raw reading. Tell the sim about both so it reports pivot rotations directly.
    var encoderSim = pivotEncoder.getSimState();
    encoderSim.Orientation = ChassisReference.Clockwise_Positive;
    encoderSim.SensorOffset = IntakeConstants.CANCODER_OFFSET_ROTATIONS;
    pivotSim.setState(Units.rotationsToRadians(IntakeConstants.PIVOT_STOWED_ROTATIONS), 0.0);
    updatePivotSimSensors();
  }

  @Override
  public void simulationPeriodic() {
    var pivotSimState = pivotMotor.getSimState();
    pivotSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    pivotSim.setInputVoltage(pivotSimState.getMotorVoltage());
    pivotSim.update(0.02);
    updatePivotSimSensors();
  }

  private void updatePivotSimSensors() {
    double pivotRotations = pivotSim.getAngularPositionRotations();
    double pivotRps = pivotSim.getAngularVelocityRPM() / 60.0;
    var pivotSimState = pivotMotor.getSimState();
    pivotSimState.setRawRotorPosition(pivotRotations * PIVOT_TOTAL_GEAR_RATIO);
    pivotSimState.setRotorVelocity(pivotRps * PIVOT_TOTAL_GEAR_RATIO);
    var encoderSim = pivotEncoder.getSimState();
    encoderSim.setRawPosition(pivotRotations * IntakeConstants.SENSOR_TO_MECHANISM_RATIO);
    encoderSim.setVelocity(pivotRps * IntakeConstants.SENSOR_TO_MECHANISM_RATIO);
  }
}
