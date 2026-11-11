package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShootComConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RollerFloor;
import frc.robot.subsystems.Shooter;
import org.littletonrobotics.junction.Logger;

/**
 * Shoots from a fixed distance without aiming. Spins up the shooter and sets the hood from the
 * distance tables, then starts feeding once the shooter is at speed (or after a timeout). When
 * released, it leaves the shooter spinning slowly and redeploys the intake.
 */
public class StaticShootCommand extends Command {

  private static final double FORCE_FIRE_TIMEOUT_S = 1.0;

  private final Shooter shooter;
  private final Hood hood;
  private final Feeder feeder;
  private final RollerFloor rollerFloor;
  private final Intake intake;
  private final double distanceM;
  private final Timer startTimer = new Timer();
  private boolean firingStarted;

  public StaticShootCommand(
      Shooter shooter,
      Hood hood,
      Feeder feeder,
      RollerFloor rollerFloor,
      Intake intake,
      double distanceM) {
    this.shooter = shooter;
    this.hood = hood;
    this.feeder = feeder;
    this.rollerFloor = rollerFloor;
    this.intake = intake;
    this.distanceM = distanceM;
    addRequirements(shooter, hood, feeder, rollerFloor, intake);
  }

  @Override
  public void initialize() {
    firingStarted = false;
    startTimer.restart();
    shooter.runVelocity(ShootComConstants.SHOOT_VELOCITY_LUT.get(distanceM));
    hood.setAngle(ShootComConstants.HOOD_ANGLE_LUT.get(distanceM));
  }

  @Override
  public void execute() {
    shooter.runVelocity(ShootComConstants.SHOOT_VELOCITY_LUT.get(distanceM));
    hood.setAngle(ShootComConstants.HOOD_ANGLE_LUT.get(distanceM));

    double shooterError = Math.abs(shooter.getVelocity() - shooter.getVelocitySetpoint());
    boolean rpsReady = shooterError <= ShootComConstants.SHOOTER_READY_RPS_TOLERANCE;
    boolean timedOut = startTimer.hasElapsed(FORCE_FIRE_TIMEOUT_S);

    if (!firingStarted && (rpsReady || timedOut)) {
      firingStarted = true;
    }

    if (firingStarted) {
      rollerFloor.runForward();
      feeder.runForward();
      intake.slowPartialStow();
    }

    Logger.recordOutput(
        "StaticShootCommand/TargetRPS", ShootComConstants.SHOOT_VELOCITY_LUT.get(distanceM));
    Logger.recordOutput("StaticShootCommand/ShooterErrorRPS", shooterError);
    Logger.recordOutput("StaticShootCommand/FiringStarted", firingStarted);
    Logger.recordOutput("StaticShootCommand/ForceFiredByTimeout", timedOut && !rpsReady);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.runVolts(ShootComConstants.SHOOTER_IDLE_VOLTS);
    hood.retract();
    feeder.stop();
    rollerFloor.stop();
    intake.deploy();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
