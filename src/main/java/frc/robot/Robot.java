package frc.robot;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.LimelightHelpers;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Main robot class. AdvantageKit is used only as a logger here: everything passed to {@code
 * Logger.recordOutput} is written to a .wpilog file on the robot and published over NetworkTables
 * for AdvantageScope. There is no log replay.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private final RobotContainer robotContainer;

  public Robot() {
    // Setup 3A Limelights
    LimelightHelpers.setupPortForwardingUSB(5); // limelight-left
    LimelightHelpers.setupPortForwardingUSB(3); // limelight-right

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Write a log file on the real robot, and always publish to NetworkTables for live viewing.
    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
    }
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();

    // Disable LiveWindow to prevent SmartDashboard from sending large NT4 updates every loop.
    LiveWindow.disableAllTelemetry();

    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    // Run the command scheduler, which polls buttons and runs all scheduled commands
    CommandScheduler.getInstance().run();

    // Update hub shift state and publish it to the dashboard and logs
    HubShiftUtil.getInstance().update();

    // Log 3D component poses for AdvantageScope visualization
    robotContainer.updateVisualization();
  }

  @Override
  public void disabledInit() {
    robotContainer.resetSimulationField();
  }

  @Override
  public void autonomousInit() {
    // PathPlanner resets the pose to the path start, so vision can start correcting right away.
    robotContainer.notifyAutoStart();

    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    // Cancel any still-running autonomous command when teleop starts
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void simulationPeriodic() {
    robotContainer.updateSimulation();
  }
}
