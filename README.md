# FRC Rebuilt 2026

Robot code for our 2026 FRC season (REBUILT), written in Java with WPILib's command-based framework.

## Robot

- **Drive:** swerve (Kraken X60s, CANcoders, Pigeon 2) using CTRE's Phoenix 6 swerve API
- **Intake:** over-the-bumper pivoting intake with a roller
- **Roller floor + feeder:** move fuel from the hopper up into the shooter
- **Shooter:** 4-inch drum driven by four Kraken X60s, plus an adjustable hood
- **Vision:** one Limelight reading AprilTags to correct odometry

## Code layout

```
src/main/java/frc/robot/
  Robot.java            Starts logging and runs the command scheduler
  RobotContainer.java   Creates subsystems, binds controller buttons, sets up autos
  subsystems/           One class per mechanism; each talks to its own motors directly
  commands/             Driving and shooting commands
  constants/            CAN IDs and tuning values
  generated/            Swerve constants (Phoenix Tuner X)
  sim/                  maple-sim physics: drivetrain and game piece simulation
  util/                 Hub shift timing and Phoenix helpers
```

## Logging

[AdvantageKit](https://docs.advantagekit.org) is used **only for logging**: values passed to
`Logger.recordOutput` are saved to a `.wpilog` file on the robot and published over NetworkTables,
so they can be viewed live or afterwards in [AdvantageScope](https://docs.advantagescope.org).
There are no IO layers and no log replay.

## Controls (PS5 controller)

| Button       | Action                                                  |
| ------------ | ------------------------------------------------------- |
| Left stick   | Drive (field relative)                                  |
| Right stick  | Rotate                                                  |
| L1 (hold)    | Aim at the hub (or a pass zone) and shoot while driving |
| R1 (hold)    | Lock heading parallel to the field walls                |
| Cross        | Deploy intake                                           |
| Triangle     | Partially stow intake                                   |
| Square (hold)| Shoot from 2 m without aiming                           |
| Circle (hold)| Shoot from 3.5 m without aiming                         |
| D-pad down   | Reverse everything to clear jams                        |
| Options      | Reset heading                                           |

## Simulation

Run `./gradlew simulateJava` (or "Simulate Robot Code" in VS Code). The simulation uses
[maple-sim](https://github.com/Shenzhen-Robotics-Alliance/maple-sim) for a physics drivetrain on the
REBUILT field, with fuel that the intake can pick up and the shooter can launch into the hub. Open
AdvantageScope and connect to the simulator to see the robot, fuel, and shot trajectories.

Autonomous routines are made in [PathPlanner](https://pathplanner.dev) and live in
`src/main/deploy/pathplanner`.
