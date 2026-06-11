# 604X — BC Provincial Championship Code

Competition code for **VRC Team 604X** for the **2023–2024 VEX Over Under** season, run at the **BC Provincial Championship**. Built on [PROS](https://pros.cs.purdue.edu/) with [LemLib](https://github.com/LemLib/LemLib) for odometry-based motion control.

## Robot Overview

| Subsystem | Hardware |
|---|---|
| Drivetrain | 6-motor tank drive, blue (600 RPM) cartridges geared to **450 RPM**, 2.75" omni wheels, 11.15" track width |
| Catapult | 1 motor with a Rotation sensor (port 12) for automatic re-cocking |
| Intake | 1 motor (port 1) |
| Front wings | Pneumatic (ADI H) — widens the robot for pushing triballs |
| Back wings | Pneumatic (ADI G) — corner descores and match-load flicks |
| Hang | Pneumatic release (ADI F), doubles as a plow in skills |
| Odometry | IMU (port 21) + one vertical 2.75" tracking wheel on a Rotation sensor (port 15) |

## Project Structure

| File | Purpose |
|---|---|
| `src/main.cpp` | Hardware configuration (motors, sensors, pneumatics), LemLib chassis/PID setup, competition callbacks, and driver control |
| `src/Autonomous.cpp` | Every autonomous routine, one function per route |
| `src/functions.cpp` | Shared helpers: drive primitives, in-house PID drive/turn (pre-LemLib), timed intake/catapult helpers |
| `include/autons.hpp` | Declarations for the autonomous routines |
| `include/functions.hpp` | Declarations for the shared helpers |

## Autonomous Routines

Routines live in `src/Autonomous.cpp`. The active one is selected with a single call inside `autonomous()` in `src/main.cpp`:

```cpp
void autonomous() {
    farSideAuton(); // swap for any routine in include/autons.hpp
}
```

### Competition routines

| Routine | Description |
|---|---|
| `farSideAuton()` | **Ran at Provincials.** Safe far-side route: intakes the ball in front, sweeps the corner triball out with the back wings, double-rams the goal, then collects and pushes the two middle triballs in with the front wings |
| `awpAuton()` | Win-point route: scores the alliance triball, descores the corner with the back wings, touches the hang bar |
| `skillsAuton()` | 30-second catapult volley from the match-load bar, then crosses the field and wing-pushes triballs into the far goal |
| `oneMinSkillsAuton()` | One-minute skills variant using the hang piston as a plow |

### Legacy / practice routines

`oldFarSideAuton()`, `oldAwpAuton()`, `backWingAwpAuton()`, `safeAwpAuton()`, and `goalPushFragment()` are earlier revisions kept compiled and ready as fallbacks.

## Driver Controls

| Control | Action |
|---|---|
| Left stick (Y) | Drive forward / backward |
| Right stick (X) | Turn (scaled to 80% for finer control) |
| L1 (hold) | Intake |
| R1 (hold) | Outtake |
| L2 (toggle) | Front wings |
| R2 (toggle) | Back wings |
| A (toggle) | Hang piston |
| Y (hold) | Fire catapult manually |

The catapult **auto-reloads**: whenever its rotation sensor reads below the cocked angle, the motor pulls the slip-gear back down — the driver only intervenes with `Y` for manual volleys.

## Motion Control

Driving in autonomous uses LemLib's boomerang controller (`moveToPose`) and `moveToPoint`, tuned with:

- **Lateral PID:** kP 15, kD 10, slew 20
- **Angular PID:** kP 1.7, kD 8, slew 20

Positions are tracked by fusing the IMU with the vertical tracking wheel at 100 Hz. The robot's live pose (X, Y, heading) is printed to the brain screen for tuning.

`src/functions.cpp` also contains the team's original in-house PID drive/turn implementations from before the switch to LemLib, still used for timed catapult/intake actions during skills.

## Building

Requires the [PROS CLI](https://pros.cs.purdue.edu/v5/getting-started/) (kernel 3.8.x, this project uses LemLib 0.5.0-rc.5):

```sh
pros make          # build
pros upload        # upload to the V5 brain
pros terminal      # view telemetry
```

## Credits

- [LemLib](https://github.com/LemLib/LemLib) — odometry and motion control library
- [PROS](https://pros.cs.purdue.edu/) — V5 RTOS and toolchain by Purdue ACM SIGBots
