package frc.robot.constants;

/** Tuning constants for the intake pivot and roller. CAN IDs are in {@link DeviceConstants}. */
public final class IntakeConstants {

  // Gear ratios
  // Motor to CANcoder shaft: 37.5:1, then CANcoder shaft to pivot axle: 2:1, total 75:1
  public static final double ROTOR_TO_SENSOR_RATIO = 37.5;
  public static final double SENSOR_TO_MECHANISM_RATIO = 2.0;
  public static final double ROLLER_GEAR_RATIO = 0.5;

  // Pivot setpoints in pivot axle rotations
  // 0.0 = fully deployed, 0.16 = partial stow, 0.33 = fully stowed
  public static final double PIVOT_DEPLOYED_ROTATIONS = 0.0;
  public static final double PIVOT_PARTIAL_STOW_ROTATIONS = 0.18;
  public static final double PIVOT_STOWED_ROTATIONS = 0.33;
  public static final double PIVOT_AT_GOAL_TOLERANCE_ROTATIONS = 0.02;

  // CANcoder absolute offset
  public static final double CANCODER_OFFSET_ROTATIONS = -0.123046875;

  // MotionMagic PID gains for the pivot (Slot 0)
  public static final double PIVOT_KP = 90.0;
  public static final double PIVOT_KI = 0.0;
  public static final double PIVOT_KD = 3.0;
  public static final double PIVOT_KS = 0.32; // static friction (V)
  public static final double PIVOT_KV = 0.13; // velocity feedforward (V*s/rot)
  public static final double PIVOT_KA = 0.0;
  // Gravity feedforward using Arm_Cosine. Assumes 0.0 rot = arm horizontal.
  public static final double PIVOT_KG = 0.45;
  // MotionMagic profile in pivot axle rot/s and rot/s^2
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 1.75;
  public static final double MOTION_MAGIC_ACCELERATION = 8.0;

  // Slow retract rate during shooting to push fuel forward without jamming
  public static final double SLOW_PARTIAL_STOW_RATE_ROT_PER_S = 0.4;

  // Voltage commands
  public static final double ROLLER_INTAKE_VOLTS = 12.0;
  public static final double ROLLER_EJECT_VOLTS = -4.0;

  // Current limits
  public static final double PIVOT_STATOR_CURRENT_LIMIT_AMPS = 40.0;
  public static final double PIVOT_SUPPLY_CURRENT_LIMIT_AMPS = 25.0;
  public static final double ROLLER_STATOR_CURRENT_LIMIT_AMPS = 40.0;
  public static final double ROLLER_SUPPLY_CURRENT_LIMIT_AMPS = 25.0; // per motor (2x = 50A)
}
