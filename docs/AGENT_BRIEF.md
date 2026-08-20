# Brief for the Claude Code agent running on the rover laptop

Read this first, then `LAPTOP_SETUP.md`. You are bringing up a physical robot that can injure
someone and that is on a hard deadline of **2026-08-25**.

## What this project is

"Unlocking the Path" (ICRA 2027) is training-free interactive navigation: the robot reaches a goal
that is unreachable *until it changes the building state* — presses a door button, calls an
elevator. The loop is `reason → ground → act → verify`, with swappable modules chosen by config.

The simulation side is finished and validated. Your job is the physical robot: an
**AgileX Ranger Mini 3.0** base, a **uFactory xArm6** arm, an **Intel RealSense D455** on a mast,
and a **Slamtec RPLIDAR A1M8**.

The minimum viable result is **one mission where a passive robot provably cannot reach the goal and
ours can, on hardware, with no human in the loop.** Everything beyond that is upside. Prefer
finishing that completely over starting anything else.

## Rules that are not negotiable

**1. Safety is layered and software is the weakest layer.** Layer 0 is the hardware E-stops on the
chassis and the arm control box. Layer 1 is the Ranger's RC transmitter, which revokes CAN command
authority below anything software can touch — **the person standing next to the robot holds the
RC**. Layer 2 is our twist mux. Never present layer 2 as sufficient.

**2. The arm interlock is not optional.** An extended arm sits ~0.88 m outside the footprint Nav2
collision-checks against. The base must not move unless the arm is stowed, verified by *measured
joint angles*, never by a state machine's belief about itself. All gates fail closed: never-seen
and stale both mean "not permitted".

**3. A gate is GREEN only when a human watched it pass.** "Should work" is not a result. Write
observations, not expectations, into `EXPERIMENT_LOG.md`.

**4. Record negative results.** The failures and the wrong theories are worth more than the
successes — they are what stops the next person repeating them. Several entries in the log exist
purely because someone chased the wrong cause for an hour.

**5. Never kill processes by a loose pattern.** On 2026-08-18 a cleanup on the workstation matched
`static_transform_publisher` by child frame name and killed 22 of the running simulation campaign's
TF publishers, invalidating trials across 21 ROS domains. Scope every kill by full command line AND
by the executable living under this repo. On the rover laptop you are alone, but the habit matters.

**6. Do not edit the simulation repo.** It works. Copy from it; never modify it in place.

## What to be suspicious of

- **A device that answers is not a device that works.** Our lidar returned correct serial number,
  firmware and health while being completely unable to scan. Identity, health, and capability are
  three separate questions.
- **Plausible-looking sensor data.** A mirrored lidar scan builds a map that looks fine and
  navigates catastrophically. Only a physical check catches it.
- **Silence.** Wrong baud on the lidar gives no data and no error. A missing TF makes the costmap
  drop every scan silently. Absence of an error message is not evidence of correctness.
- **`use_sim_time`.** Everything on real hardware must run with `use_sim_time:=false`. The sim
  configs default it true, and the failure mode is nodes quietly waiting forever for a clock.

## Where things are

| | |
|---|---|
| This repo (hardware) | `~/utp_robot` — drivers, safety stack, bringup, calibration |
| Pipeline repo (sim) | `https://github.com/AnikS22/unlocking-the-path.git` |
| Running log + gates | `EXPERIMENT_LOG.md` — **update it every session** |
| Provisioning | `docs/LAPTOP_SETUP.md` |
| Every spec, ID and pinned version | `docs/HARDWARE_SPECS.md` |
| Calibration procedures | `docs/CALIBRATION.md` |

## The order of work

Sequence by what can *falsify the plan* first, dependency second:

1. Laptop provisioned, drivers built, `/scan` live (`LAPTOP_SETUP.md`)
2. **Site survey** — are the ADA doors motion-activated? If they open without a press, `passive`
   succeeds and the whole mission measures nothing. One visit falsifies it. Highest value per hour
   of anything in this project.
3. CAN up, base moves, **and the `/cmd_vel` stale-command timeout verified** — if the driver holds
   the last twist when its publisher dies, that is a runaway and everything stops until there is a
   watchdog.
4. Calibration in the dependency order in `CALIBRATION.md`
5. Safety stack on real hardware
6. Mission R0, then R1 to completion

Do not parallelise 2 and 6. Finish R1.
