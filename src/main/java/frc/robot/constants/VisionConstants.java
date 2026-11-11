package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Constants for vision: camera names, filter thresholds, and camera positions on the robot. */
public final class VisionConstants {

  // Limelight name as set on its web interface
  public static final String LIMELIGHT_NAME = "limelight-back";

  // Pose estimation filter thresholds
  public static final int MIN_TAG_COUNT = 1;
  public static final double MAX_POSE_DISTANCE_METERS = 5.0;
  // Skip readings if the robot is spinning too fast (MegaTag2 accuracy drops with yaw rate)
  public static final double MAX_VISION_OMEGA_RAD_PER_SEC = 2.0;
  // Skip readings if the robot is moving too fast to avoid blur errors
  public static final double MAX_VISION_SPEED_MPS = 3.0;
  // Max distance to accept a tag reading depending on how many tags are visible
  public static final double MAX_SINGLE_TAG_DISTANCE_M = 4.0;
  public static final double MAX_MULTI_TAG_DISTANCE_M = 6.0;
  // Ambiguity limits. Single-tag is stricter since it is harder to solve accurately.
  public static final double MAX_SINGLE_TAG_AMBIGUITY = 0.10;
  public static final double MAX_MULTI_TAG_AMBIGUITY = 0.25;
  // Max distance the vision pose can be from our odometry before we reject it
  public static final double MAX_PLAUSIBLE_JUMP_SINGLE = 0.4;
  public static final double MAX_PLAUSIBLE_JUMP_MULTI = 0.8;
  // Seconds without a good reading before we hard-reset the pose from vision
  public static final double STALE_SEED_TIMEOUT_SECONDS = 2.0;
  // Minimum quality score needed to do a hard pose reset (needs at least 2 tags)
  public static final double MIN_RESEED_SCORE = 1.5;
  // How much to trust vision based on tag distance. Farther tags are less trusted.
  // Multi-tag coefficients already include a 0.6× confidence factor vs raw 0.005/0.01.
  public static final double MULTI_TAG_XY_STD_DEV_COEFFICIENT = 0.003; // = 0.005 × 0.6
  // Single-tag is noisier so it gets a higher (less trusted) coefficient
  public static final double SINGLE_TAG_XY_STD_DEV_COEFFICIENT = 0.02;
  // Heading trust: multi-tag uses distance scaling, single-tag heading is fully ignored
  public static final double MULTI_TAG_ROT_STD_DEV_COEFFICIENT = 0.006; // = 0.01 × 0.6
  public static final double VISION_THETA_STD_DEV = 9999.0; // single-tag heading not trusted
  // Clamp tag distance to avoid crazy values in the noise formula
  public static final double MAX_TAG_DISTANCE_FOR_STD_DEV = 7.0; // meters
  // Maximum |Z| from MegaTag2 3D pose solve — robot is on the ground (bumps are ~16.5 cm tall)
  public static final double MAX_POSE_Z_METERS = 0.5;

  // Simulation camera settings. Based on LL3A specs.
  public static final double CAMERA_FOV_DEGREES = 68.8;
  public static final double CAMERA_VFOV_DEGREES = 49.7;
  public static final double MAX_DETECTION_DISTANCE_METERS = 7.0;

  // Camera positions on the robot. X forward, Y left, Z up (WPILib convention).
  // Rotation3d(roll, pitch, yaw) where identity means the camera faces forward.
  public static final Transform3d ROBOT_TO_LIMELIGHT_BACK =
      new Transform3d(
          new Translation3d(-0.321, 0.0, 0.412),
          new Rotation3d(
              0.0, Math.toRadians(-20.0), Math.toRadians(180))); // faces back, pitched up 20 deg
}
