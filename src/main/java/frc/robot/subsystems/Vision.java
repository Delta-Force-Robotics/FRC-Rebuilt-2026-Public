package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.littletonrobotics.junction.Logger;

/**
 * Reads AprilTag pose estimates from the back Limelight and sends accepted measurements to the
 * drivetrain's pose estimator.
 *
 * <p>Each loop the camera reading is checked against a set of filters. Accepted readings are added
 * as soft measurements. The pose is only hard-reset on the first good reading, or when vision has
 * been lost for a while or odometry has left the field, and only with a solid multi-tag reading.
 *
 * <p>In simulation the camera is faked from the true maple-sim robot pose, with noise added.
 */
public class Vision extends SubsystemBase {
  private static final String LOG_PREFIX = "Vision/" + VisionConstants.LIMELIGHT_NAME + "/";

  // Simulated camera noise
  private static final double SIM_XY_NOISE_M_PER_M = 0.02; // XY noise per meter of tag distance
  private static final double SIM_HEADING_NOISE_DEG = 0.5;
  private static final double SIM_LATENCY_SECONDS = 0.05;

  /** One pose estimate from the camera. */
  private static class Reading {
    boolean hasTarget = false;
    int tagCount = 0;
    Pose2d pose = Pose2d.kZero;
    double poseZ = 0.0; // non-zero means the 3D solve is wrong (robot should be on the ground)
    double captureTimestamp = 0.0; // FPGA time the frame was captured
    double avgTagDistance = 0.0;
    double worstTagAmbiguity = 1.0; // highest ambiguity of any visible tag
    Pose3d[] visibleTagPoses = new Pose3d[0];
  }

  private static final double[] EMPTY = {};

  private final Drive drive;

  // Limelight NetworkTables entries
  private final NetworkTable limelightTable =
      NetworkTableInstance.getDefault().getTable(VisionConstants.LIMELIGHT_NAME);
  private final DoubleArraySubscriber botposeEntry =
      limelightTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(EMPTY);
  private final DoubleSubscriber tvEntry = limelightTable.getDoubleTopic("tv").subscribe(0.0);
  private final DoubleArraySubscriber rawFiducialsEntry =
      limelightTable.getDoubleArrayTopic("rawfiducials").subscribe(EMPTY);
  private final Random simRandom = new Random();

  // Odometry seeding state
  private boolean hasSeededFromVision = false;
  private double lastVisionUpdateTimestamp = 0.0;

  public Vision(Drive drive) {
    this.drive = drive;
  }

  /**
   * Marks vision as already seeded, for example when PathPlanner resets the pose at auto start.
   * This lets vision start correcting right away without overwriting the preset pose.
   */
  public void markSeeded() {
    hasSeededFromVision = true;
    lastVisionUpdateTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void periodic() {
    Reading reading = RobotBase.isSimulation() ? readSimulatedCamera() : readLimelight();
    logReading(reading);

    ChassisSpeeds speeds = drive.getChassisSpeeds();
    double robotSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    Pose2d currentPose = drive.getPose();
    double now = Timer.getFPGATimestamp();
    boolean odometryBad = !isPoseInsideField(currentPose);
    boolean isStale =
        (now - lastVisionUpdateTimestamp) > VisionConstants.STALE_SEED_TIMEOUT_SECONDS;

    Matrix<N3, N1> stdDevs =
        evaluate(reading, speeds.omegaRadiansPerSecond, robotSpeed, currentPose, isStale);
    Logger.recordOutput("Vision/Accepted", stdDevs != null);

    if (stdDevs != null) {
      double score = score(reading);
      boolean solidReading = reading.tagCount >= 2 && score >= VisionConstants.MIN_RESEED_SCORE;
      Logger.recordOutput("Vision/Score", score);

      if (!hasSeededFromVision) {
        // First seed: only hard-reset when we have a solid multi-tag reading.
        if (solidReading) {
          drive.resetPose(reading.pose);
          hasSeededFromVision = true;
          lastVisionUpdateTimestamp = now;
        }
      } else {
        // Hard-reset when vision has been lost for too long or odometry has left the field.
        // In both cases the pose estimator is in a bad state and a soft correction won't fix it.
        if ((isStale || odometryBad) && solidReading) {
          drive.resetPose(reading.pose);
        } else {
          drive.addVisionMeasurement(reading.pose, reading.captureTimestamp, stdDevs);
        }
        lastVisionUpdateTimestamp = now;
      }
    }

    Logger.recordOutput("Vision/OdometryHealthy", !odometryBad);
  }

  /**
   * Checks a reading against all the filters. Returns the measurement standard deviations if it
   * should be used, or null if it should be ignored.
   */
  private Matrix<N3, N1> evaluate(
      Reading in, double omega, double robotSpeed, Pose2d currentPose, boolean isStale) {
    if (!in.hasTarget || in.tagCount == 0) return null;

    // Reject poses outside the field
    if (!isPoseInsideField(in.pose)) return null;

    // Reject if the 3D solve placed the robot off the ground (corrupted solve)
    if (Math.abs(in.poseZ) > VisionConstants.MAX_POSE_Z_METERS) return null;

    // Emergency recovery: odometry is outside the field or vision has been stale for too long.
    // In either case, relax the filters so recovery can actually happen.
    boolean odometryBad = !isPoseInsideField(currentPose);
    boolean emergencyRecovery =
        ((hasSeededFromVision && isStale) || odometryBad) && in.tagCount >= 2;

    // Reject if rotating or moving too fast (blur and timing lag hurt accuracy),
    // unless odometry is bad and we need vision to recover.
    if (!odometryBad && Math.abs(omega) > VisionConstants.MAX_VISION_OMEGA_RAD_PER_SEC) {
      return null;
    }
    if (!odometryBad && robotSpeed > VisionConstants.MAX_VISION_SPEED_MPS) return null;

    // Distance limits. Single-tag is too noisy at long range even for recovery.
    if (in.tagCount < 2 && in.avgTagDistance > VisionConstants.MAX_SINGLE_TAG_DISTANCE_M) {
      return null;
    }
    if (!odometryBad
        && in.tagCount >= 2
        && in.avgTagDistance > VisionConstants.MAX_MULTI_TAG_DISTANCE_M) {
      return null;
    }

    // Ambiguity check using the worst tag. Skipped during emergency recovery.
    if (!emergencyRecovery) {
      double maxAmbiguity =
          in.tagCount < 2
              ? VisionConstants.MAX_SINGLE_TAG_AMBIGUITY
              : VisionConstants.MAX_MULTI_TAG_AMBIGUITY;
      if (in.worstTagAmbiguity > maxAmbiguity) return null;
    }

    // Reject if too far from where odometry thought we were when the frame was captured.
    // Skipped when odometry is outside the field, since that odometry pose is meaningless.
    if (hasSeededFromVision && !odometryBad) {
      Pose2d poseAtCapture = drive.samplePoseAt(in.captureTimestamp).orElse(currentPose);
      double distFromCapture = in.pose.getTranslation().getDistance(poseAtCapture.getTranslation());
      double maxJump =
          (in.tagCount >= 2
                  ? VisionConstants.MAX_PLAUSIBLE_JUMP_MULTI
                  : VisionConstants.MAX_PLAUSIBLE_JUMP_SINGLE)
              + robotSpeed * 0.05;

      if (!isStale && distFromCapture > maxJump) return null;
      // If vision has been lost, still limit single-tag jumps to 2 meters
      if (isStale && in.tagCount < 2 && distFromCapture > 2.0) return null;
    }

    // Trust readings less the farther away the tags are
    double clampedDist = Math.min(in.avgTagDistance, VisionConstants.MAX_TAG_DISTANCE_FOR_STD_DEV);
    double d2 = clampedDist * clampedDist;
    double xyStdDev;
    double rotStdDev;
    if (in.tagCount >= 2) {
      xyStdDev = Math.max(VisionConstants.MULTI_TAG_XY_STD_DEV_COEFFICIENT * d2, 0.02);
      rotStdDev = Math.max(VisionConstants.MULTI_TAG_ROT_STD_DEV_COEFFICIENT * d2, 0.05);
    } else {
      xyStdDev = Math.max(VisionConstants.SINGLE_TAG_XY_STD_DEV_COEFFICIENT * d2, 0.05);
      rotStdDev = VisionConstants.VISION_THETA_STD_DEV; // single-tag heading is not reliable
    }
    return VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev);
  }

  /** Quality score: more tags and closer is better, higher ambiguity is worse. */
  private static double score(Reading in) {
    return (in.tagCount * 2.0) - (in.avgTagDistance * 0.4) - (in.worstTagAmbiguity * 3.0);
  }

  private static boolean isPoseInsideField(Pose2d pose) {
    return pose.getX() > 0.0
        && pose.getX() < Constants.FIELD_LENGTH_METERS
        && pose.getY() > 0.0
        && pose.getY() < Constants.FIELD_WIDTH_METERS;
  }

  /** Reads the MegaTag1 (6DOF) robot pose from the Limelight over NetworkTables. */
  private Reading readLimelight() {
    Reading in = new Reading();

    // botpose_wpiblue: x, y, z, roll, pitch, yaw, latency_ms, tagCount, tagSpan, avgDist, avgAmb
    double[] botpose = botposeEntry.get(EMPTY);
    in.hasTarget = tvEntry.get(0.0) > 0.5 && botpose.length >= 7;
    if (in.hasTarget) {
      in.pose = new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));
      in.poseZ = botpose[2];
      in.captureTimestamp = Timer.getFPGATimestamp() - botpose[6] / 1000.0;
      if (botpose.length >= 8) in.tagCount = (int) botpose[7];
      if (botpose.length >= 10) in.avgTagDistance = botpose[9];
    }

    // rawfiducials holds 7 values per visible tag: id, txnc, tync, ta, distToCamera,
    // distToRobot, ambiguity. Find the worst ambiguity, and the tag poses for visualization.
    // With no per-tag data, keep the worst-case ambiguity so the reading fails the filter.
    double[] raw = rawFiducialsEntry.get(EMPTY);
    List<Pose3d> tagPoses = new ArrayList<>();
    if (raw.length >= 7) in.worstTagAmbiguity = 0.0;
    for (int i = 0; i + 7 <= raw.length; i += 7) {
      in.worstTagAmbiguity = Math.max(in.worstTagAmbiguity, raw[i + 6]);
      Constants.FIELD_LAYOUT.getTagPose((int) raw[i]).ifPresent(tagPoses::add);
    }
    in.visibleTagPoses = tagPoses.toArray(new Pose3d[0]);
    return in;
  }

  /**
   * Fakes a camera reading from the true simulated robot pose, using the camera's field of view.
   */
  private Reading readSimulatedCamera() {
    Reading in = new Reading();
    var driveSim = drive.getDriveSimulation();
    if (driveSim == null) return in;

    Pose3d robotPose = new Pose3d(driveSim.getSimulatedDriveTrainPose());
    Pose3d cameraPose = robotPose.transformBy(VisionConstants.ROBOT_TO_LIMELIGHT_BACK);
    double hFov = Math.toRadians(VisionConstants.CAMERA_FOV_DEGREES);
    double vFov = Math.toRadians(VisionConstants.CAMERA_VFOV_DEGREES);

    // Find all AprilTags the camera can see
    List<Pose3d> visible = new ArrayList<>();
    double totalDistance = 0.0;
    for (AprilTag tag : Constants.FIELD_LAYOUT.getTags()) {
      double distance = cameraPose.getTranslation().getDistance(tag.pose.getTranslation());
      if (distance > VisionConstants.MAX_DETECTION_DISTANCE_METERS) continue;

      // Skip tags facing away from the camera
      Translation3d tagNormal = new Translation3d(1, 0, 0).rotateBy(tag.pose.getRotation());
      Translation3d tagToCamera = cameraPose.getTranslation().minus(tag.pose.getTranslation());
      if (tagNormal.dot(tagToCamera) <= 0.0) continue;

      // Skip tags outside the field of view
      Pose3d tagInCamera = tag.pose.relativeTo(cameraPose);
      if (tagInCamera.getX() <= 0.0) continue;
      if (Math.atan2(Math.abs(tagInCamera.getY()), tagInCamera.getX()) > hFov / 2.0) continue;
      if (Math.atan2(Math.abs(tagInCamera.getZ()), tagInCamera.getX()) > vFov / 2.0) continue;

      visible.add(tag.pose);
      totalDistance += distance;
    }
    if (visible.isEmpty()) return in;

    // Add more noise when tags are farther away
    in.hasTarget = true;
    in.tagCount = visible.size();
    in.avgTagDistance = totalDistance / visible.size();
    double xySigma = SIM_XY_NOISE_M_PER_M * in.avgTagDistance;
    Pose2d truePose = robotPose.toPose2d();
    in.pose =
        new Pose2d(
            truePose.getX() + simRandom.nextGaussian() * xySigma,
            truePose.getY() + simRandom.nextGaussian() * xySigma,
            truePose
                .getRotation()
                .plus(Rotation2d.fromDegrees(simRandom.nextGaussian() * SIM_HEADING_NOISE_DEG)));
    in.worstTagAmbiguity = 0.0;
    in.captureTimestamp = Timer.getFPGATimestamp() - SIM_LATENCY_SECONDS;
    in.visibleTagPoses = visible.toArray(new Pose3d[0]);
    return in;
  }

  /** Logs the reading, plus camera-to-tag rays for AdvantageScope. */
  private void logReading(Reading in) {
    Logger.recordOutput(LOG_PREFIX + "HasTarget", in.hasTarget);
    Logger.recordOutput(LOG_PREFIX + "TagCount", in.tagCount);
    Logger.recordOutput(LOG_PREFIX + "Pose", in.pose);
    Logger.recordOutput(LOG_PREFIX + "PoseZ", in.poseZ);
    Logger.recordOutput(LOG_PREFIX + "CaptureTimestamp", in.captureTimestamp);
    Logger.recordOutput(LOG_PREFIX + "AvgTagDistance", in.avgTagDistance);
    Logger.recordOutput(LOG_PREFIX + "WorstTagAmbiguity", in.worstTagAmbiguity);

    Pose3d cameraPose =
        new Pose3d(drive.getPose()).transformBy(VisionConstants.ROBOT_TO_LIMELIGHT_BACK);
    Pose3d[] rays = new Pose3d[in.visibleTagPoses.length * 2];
    for (int i = 0; i < in.visibleTagPoses.length; i++) {
      rays[i * 2] = cameraPose;
      rays[i * 2 + 1] = in.visibleTagPoses[i];
    }
    Logger.recordOutput(LOG_PREFIX + "CameraPose", cameraPose);
    Logger.recordOutput(LOG_PREFIX + "TagRays", rays);
  }
}
