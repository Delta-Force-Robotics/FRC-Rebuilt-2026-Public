package frc.robot.constants;

/** All CAN device IDs, grouped by subsystem. Edit this file to reassign IDs. */
public final class DeviceConstants {

  public static final class Intake {
    public static final int PIVOT_MOTOR_ID = 61; // CANivore "ctre"
    public static final int ROLLER_MOTOR_ID = 1; // RoboRIO
    public static final int ROLLER_FOLLOWER_MOTOR_ID = 2; // RoboRIO
    public static final int PIVOT_CANCODER_ID = 60; // CANivore "ctre"
  }

  public static final class RollerFloor {
    public static final int MOTOR_ID = 30; // CANivore "ctre"
  }

  public static final class Feeder {
    public static final int LEAD_MOTOR_ID = 40; // RoboRIO (feeder bottom)
    public static final int FOLLOWER_MOTOR_ID = 41; // RoboRIO (feeder top)
  }

  public static final class Hood {
    public static final int MOTOR_ID = 50; // RoboRIO
  }

  public static final class Shooter {
    public static final int LEADER_ID = 51; // RoboRIO (leader, Top Left)
    public static final int FOLLOWER_1_ID = 52; // RoboRIO (same direction as leader, Bottom Left)
    public static final int FOLLOWER_2_ID = 53; // RoboRIO (reversed, Top Right)
    public static final int FOLLOWER_3_ID = 54; // RoboRIO (reversed, Bottom Right)
  }
}
