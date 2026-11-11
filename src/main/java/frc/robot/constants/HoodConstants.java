package frc.robot.constants;

/** Tuning constants for the hood. CAN IDs are in {@link DeviceConstants}. */
public final class HoodConstants {

  // Gear ratio: motor -> driving gear: 3:1
  public static final double TOTAL_GEAR_RATIO = 3.0;

  // Angle limits in hood mechanism (driving gear) rotations
  // 0.0 = fully stowed; 1.4 = max elevation
  public static final double MIN_ANGLE_ROTATIONS = 0.0;
  public static final double MAX_ANGLE_ROTATIONS = 1.4;
  public static final double AT_GOAL_TOLERANCE_ROTATIONS = 0.005;

  // PositionVoltage PID (Slot 0)
  public static final double KP = 39.0;
  public static final double KI = 8.5;
  public static final double KD = 0.5;
  public static final double KS = 0.30;
  public static final double KV = 0.15;

  // Current limits
  public static final double STATOR_CURRENT_LIMIT_AMPS = 40.0;
  public static final double SUPPLY_CURRENT_LIMIT_AMPS = 25.0;
}
