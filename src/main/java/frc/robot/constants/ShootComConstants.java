package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Constants for the shoot command: target positions, LUTs, heading PID, and readiness thresholds.
 */
public final class ShootComConstants {

  // Target positions on the blue side in meters (WPILib field origin)
  public static final Translation2d HUB_BLUE = new Translation2d(4.6, 4.05);
  // Pass zone targets used when the robot is outside the shoot zone
  public static final Translation2d PASS_ZONE_1_BLUE = new Translation2d(3.0, 2.0);
  public static final Translation2d PASS_ZONE_2_BLUE = new Translation2d(3.0, 6.0);

  // Shoot zone: robot targets the hub when on the blue side (X under 4.5 m)
  public static final double SHOOT_ZONE_X_MAX = 4.5;
  public static final double SHOOT_ZONE_Y_MIN = 0.0;
  public static final double SHOOT_ZONE_Y_MAX = Constants.FIELD_WIDTH_METERS;

  // Y boundary between pass zone 1 (below) and pass zone 2 (above)
  // Y coordinates are the same on both alliances in WPILib coordinates
  public static final double PASS_ZONE_Y_THRESHOLD = 4.03;

  // Heading PID constants
  public static final double HEADING_KP = 7.0;
  public static final double HEADING_KD = 0.6;
  public static final double HEADING_MAX_VEL_RAD_PER_S = 10.0;
  public static final double HEADING_MAX_ACCEL_RAD_PER_S2 = 35.0;

  // Readiness thresholds. Feeder and roller floor only run when both conditions are met.
  /** Heading error threshold (radians) before feeding starts (~6°). */
  public static final double HEADING_TOLERANCE_RADIANS = 0.15;

  /** Allowed shooter velocity error (RPS) relative to setpoint before feeding starts (~5%). */
  public static final double SHOOTER_READY_RPS_TOLERANCE = 5.0;
  /** Shooter voltage applied after command ends, keeping flywheel warm for the next shot. */
  public static final double SHOOTER_IDLE_VOLTS = 2.0;
  /** Max translation speed as a fraction of full speed while ShootCommand is active. */
  public static final double SHOOT_DRIVE_SPEED_FACTOR = 0.25;

  // LUT data: distance in meters to shooter speed in RPS
  public static final double[][] SHOOT_VELOCITY_LUT_DATA = {
    {1, 25.0},
    {1.5, 26.0},
    {2.0, 27.0},
    {2.5, 29.0},
    {3.0, 31.4},
    {3.5, 31.4},
    {4.0, 34.4},
    {4.5, 34.6},
    {5.0, 36.5},
    {6.0, 39.0}
  };

  // LUT data: distance in meters to hood angle in mechanism rotations
  public static final double[][] HOOD_ANGLE_LUT_DATA = {
    {1, 0},
    {1.5, 0.25},
    {2, 0.37},
    {2.5, 0.42},
    {3.0, 0.52},
    {3.5, 0.56},
    {4.0, 0.6},
    {4.5, 0.66},
    {5.0, 0.72},
    {6.0, 0.82}
  };

  // Lookup tables filled from the data arrays above
  public static final InterpolatingDoubleTreeMap SHOOT_VELOCITY_LUT =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap HOOD_ANGLE_LUT = new InterpolatingDoubleTreeMap();

  static {
    for (double[] entry : SHOOT_VELOCITY_LUT_DATA) {
      SHOOT_VELOCITY_LUT.put(entry[0], entry[1]);
    }
    for (double[] entry : HOOD_ANGLE_LUT_DATA) {
      HOOD_ANGLE_LUT.put(entry[0], entry[1]);
    }
  }
}
