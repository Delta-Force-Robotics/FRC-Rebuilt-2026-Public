package frc.robot.constants;

/** Tuning constants for the roller floor. CAN IDs are in {@link DeviceConstants}. */
public final class RollerFloorConstants {

  // Gear ratio: 1:3 (3 motor rotations per 1 output rotation)
  public static final double GEAR_RATIO = 3.0;

  // Voltage commands
  public static final double FORWARD_VOLTS = 8.0;

  // Current limits
  public static final double STATOR_CURRENT_LIMIT_AMPS = 40.0;
  public static final double SUPPLY_CURRENT_LIMIT_AMPS = 30.0;
}
