package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/** Robot-wide constants: CAN bus names and field dimensions. */
public final class Constants {
  public static final CANBus RIO_BUS = new CANBus("rio");
  public static final CANBus CANIVORE_BUS = new CANBus("ctre");

  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  public static final double FIELD_LENGTH_METERS = FIELD_LAYOUT.getFieldLength();
  public static final double FIELD_WIDTH_METERS = FIELD_LAYOUT.getFieldWidth();
}
