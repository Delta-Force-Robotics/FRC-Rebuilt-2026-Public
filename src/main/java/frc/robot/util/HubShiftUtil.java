package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

/**
 * Tracks hub shift timing for the 2026 REBUILT game.
 *
 * <p>The FMS sends one character at the start of teleop: 'R' means Red goes inactive first
 * (inactive in Shifts 1 and 3, active in 2 and 4). 'B' is the same but for Blue. If no data has
 * been received yet, we treat the hub as active.
 *
 * <p>Match time counts down from 140 seconds: above 130s = Transition (both hubs active) 130 to
 * 105s = Shift 1 105 to 80s = Shift 2 80 to 55s = Shift 3 55 to 30s = Shift 4 below 30s = Endgame
 * (both hubs active)
 *
 * <p>Call update() from Robot.robotPeriodic().
 */
public class HubShiftUtil {

  private static HubShiftUtil instance;

  private boolean redInactiveFirst = false;
  private boolean gameDataReceived = false;

  // Only update SmartDashboard at about 10 Hz instead of every loop to save CPU time.
  private int m_tickCount = 0;
  private static final int DASHBOARD_UPDATE_INTERVAL = 5; // every 5 ticks = 10 Hz

  private HubShiftUtil() {}

  public static HubShiftUtil getInstance() {
    if (instance == null) instance = new HubShiftUtil();
    return instance;
  }

  /** Call from Robot.robotPeriodic(). Reads game data and publishes to SmartDashboard. */
  public void update() {
    if (!gameDataReceived) {
      String raw = DriverStation.getGameSpecificMessage();
      if (raw.length() > 0) {
        char c = raw.charAt(0);
        if (c == 'R') {
          redInactiveFirst = true;
          gameDataReceived = true;
        } else if (c == 'B') {
          redInactiveFirst = false;
          gameDataReceived = true;
        }
      }
    }

    // Only push to SmartDashboard at the reduced rate
    m_tickCount++;
    if (m_tickCount < DASHBOARD_UPDATE_INTERVAL) return;
    m_tickCount = 0;

    if (DriverStation.isAutonomousEnabled()) {
      publishAuto();
    } else if (DriverStation.isTeleopEnabled()) {
      publishTeleop();
    } else {
      publishDisabled();
    }
  }

  /** Returns true if our alliance hub is currently active. */
  public boolean isHubActive() {
    Alliance alliance = DriverStation.getAlliance().orElse(null);
    if (alliance == null) return false;
    if (DriverStation.isAutonomousEnabled()) return true;
    if (!DriverStation.isTeleopEnabled()) return false;
    return computeHubActive(alliance, DriverStation.getMatchTime());
  }

  /**
   * Returns seconds until our hub becomes active again, or -1 if already active or no more active
   * windows are coming.
   */
  public double getSecondsUntilNextActiveShift() {
    Alliance alliance = DriverStation.getAlliance().orElse(null);
    if (alliance == null || !DriverStation.isTeleopEnabled()) return -1;

    double t = DriverStation.getMatchTime();
    if (computeHubActive(alliance, t)) return -1;

    double[] boundaries = {105, 80, 55, 30};
    for (double boundary : boundaries) {
      if (boundary > t) continue;
      if (computeHubActive(alliance, boundary + 0.001)) {
        return t - boundary;
      }
    }
    return -1;
  }

  /** Returns seconds until the current shift ends, regardless of alliance. */
  public double getShiftTimeRemaining() {
    if (!DriverStation.isTeleopEnabled()) return -1;

    double t = DriverStation.getMatchTime();

    double[] boundaries = {130, 105, 80, 55, 30};
    for (double boundary : boundaries) {
      if (t > boundary) {
        return t - boundary;
      }
    }
    return Math.max(t, 0);
  }

  /** True if FMS game data has been received this match. */
  public boolean isGameDataReceived() {
    return gameDataReceived;
  }

  /**
   * Returns true if our alliance is active in Shifts 1 and 3. Returns false if FMS game data has
   * not been received yet.
   */
  public boolean isActiveFirst() {
    if (!gameDataReceived) return false;
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    return alliance == Alliance.Red ? !redInactiveFirst : redInactiveFirst;
  }

  private boolean computeHubActive(Alliance alliance, double matchTime) {
    if (!gameDataReceived) return true;

    boolean shift1Active;
    if (alliance == Alliance.Red) {
      shift1Active = !redInactiveFirst;
    } else {
      shift1Active = redInactiveFirst;
    }

    if (matchTime > 130) {
      return true;
    } else if (matchTime > 105) {
      return shift1Active;
    } else if (matchTime > 80) {
      return !shift1Active;
    } else if (matchTime > 55) {
      return shift1Active;
    } else if (matchTime > 30) {
      return !shift1Active;
    } else {
      return true;
    }
  }

  private String shiftName(double matchTime) {
    if (matchTime > 130) return "TRANSITION";
    if (matchTime > 105) return "SHIFT 1";
    if (matchTime > 80) return "SHIFT 2";
    if (matchTime > 55) return "SHIFT 3";
    if (matchTime > 30) return "SHIFT 4";
    return "ENDGAME";
  }

  private void publishTeleop() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    double t = DriverStation.getMatchTime();
    boolean hubActive = computeHubActive(alliance, t);
    double shiftTimeRemaining = getShiftTimeRemaining();
    String phase = shiftName(t);
    boolean activeFirst = isActiveFirst();
    double matchTime = Math.max(t, 0);

    SmartDashboard.putString("Shift/Phase", phase);
    SmartDashboard.putBoolean("Shift/HubActive", hubActive);
    SmartDashboard.putBoolean("Shift/ActiveFirst", activeFirst);
    SmartDashboard.putBoolean("Shift/GameDataReceived", gameDataReceived);
    SmartDashboard.putNumber("Shift/MatchTimeRemaining", matchTime);
    SmartDashboard.putNumber("Shift/ShiftTimeRemaining", shiftTimeRemaining);

    Logger.recordOutput("Shift/Phase", phase);
    Logger.recordOutput("Shift/HubActive", hubActive);
    Logger.recordOutput("Shift/ActiveFirst", activeFirst);
    Logger.recordOutput("Shift/GameDataReceived", gameDataReceived);
    Logger.recordOutput("Shift/MatchTimeRemaining", matchTime);
    Logger.recordOutput("Shift/ShiftTimeRemaining", shiftTimeRemaining);
  }

  private void publishAuto() {
    double t = Math.max(DriverStation.getMatchTime(), 0);
    boolean activeFirst = isActiveFirst();

    SmartDashboard.putString("Shift/Phase", "AUTO");
    SmartDashboard.putBoolean("Shift/HubActive", true);
    SmartDashboard.putBoolean("Shift/ActiveFirst", activeFirst);
    SmartDashboard.putBoolean("Shift/GameDataReceived", gameDataReceived);
    SmartDashboard.putNumber("Shift/MatchTimeRemaining", t);
    SmartDashboard.putNumber("Shift/ShiftTimeRemaining", -1);

    Logger.recordOutput("Shift/Phase", "AUTO");
    Logger.recordOutput("Shift/HubActive", true);
    Logger.recordOutput("Shift/ActiveFirst", activeFirst);
    Logger.recordOutput("Shift/GameDataReceived", gameDataReceived);
    Logger.recordOutput("Shift/MatchTimeRemaining", t);
    Logger.recordOutput("Shift/ShiftTimeRemaining", -1.0);
  }

  private void publishDisabled() {
    boolean activeFirst = isActiveFirst();

    SmartDashboard.putString("Shift/Phase", "DISABLED");
    SmartDashboard.putBoolean("Shift/HubActive", false);
    SmartDashboard.putBoolean("Shift/ActiveFirst", activeFirst);
    SmartDashboard.putBoolean("Shift/GameDataReceived", gameDataReceived);
    SmartDashboard.putNumber("Shift/MatchTimeRemaining", 0);
    SmartDashboard.putNumber("Shift/ShiftTimeRemaining", -1);

    Logger.recordOutput("Shift/Phase", "DISABLED");
    Logger.recordOutput("Shift/HubActive", false);
    Logger.recordOutput("Shift/ActiveFirst", activeFirst);
    Logger.recordOutput("Shift/GameDataReceived", gameDataReceived);
    Logger.recordOutput("Shift/MatchTimeRemaining", 0.0);
    Logger.recordOutput("Shift/ShiftTimeRemaining", -1.0);
  }
}
