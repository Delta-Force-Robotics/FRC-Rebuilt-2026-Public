package frc.robot.constants;

/** Tuning constants for the shooter. CAN IDs are in {@link DeviceConstants}. */
public final class ShooterConstants {

  public static final double SHOOT_VOLTS = 6.0;

  // Current limits
  public static final double STATOR_CURRENT_LIMIT_AMPS = 80.0;
  public static final double SUPPLY_CURRENT_LIMIT_AMPS = 40.0; // sustained per motor (4x = 160A)
  public static final double SUPPLY_CURRENT_LOWER_LIMIT_AMPS = 60.0; // burst for spin-up
  public static final double SUPPLY_CURRENT_LOWER_TIME_SECS = 1.0; // burst duration before throttle
  public static final double FREE_SPEED_RPM = 5800;

  // Velocity PID gains (slot 0)
  public static final double VELOCITY_KP = 0.615;
  public static final double VELOCITY_KI = 0.07;
  public static final double VELOCITY_KD = 0.0;
  public static final double VELOCITY_KS = 0.27; // static friction (V)
  public static final double VELOCITY_KV = 12.0 / (FREE_SPEED_RPM / 60); // 12V / free speed in RPS
  public static final double VELOCITY_KA = 0.0; // acceleration feedforward
}
