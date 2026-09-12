# Multi-floor pipeline — state as of 2026-09-11, end of session

Resume point for the floor-5 -> floor-1 elevator run. Everything below was measured on the
robot that day, not inferred.

## The one command

```bash
cd ~/utp_robot && source bringup/env.sh
python3 bringup/run_dataset.py --scene f5_to_f1_elevator --method ours --kind full_trial \
    --map f5_lobby_20260910 --frames continuous --fps 3 --bag full \
    -- bash bringup/mission.sh --from 5 --to 1L --ready
```

`--ready` is NOT optional for a recorded run. See "Recording" below.

## Preconditions, in order

1. **SWB up on the transmitter.** The chassis silently discards every command in RC mode while
   odom, the mux and /cmd_vel all look healthy. The mission stops at stage 0 without it. It
   dropped to RC three times between runs on 2026-09-11 — check it every time.
2. **Chassis not in EXCEPTION.** `python3 bringup/claim_can.py` refuses from EXCEPTION and cannot
   clear it. With `error=0x0000` the cause is either an e-stop pressed (release BOTH the chassis
   and transmitter ones) or the transmitter off while in RC mode (lost-link failsafe — turn it on,
   leave the sticks). This is where the session ended.
3. **The right map loaded.** `cat maps/.loaded_map` must name `f5_lobby_20260910`. An aborted run
   leaves floor 1's map loaded, because the mission swaps in the background during the ride and
   that swap completes even if the run dies. This bit us once: the robot sat on floor 5 with
   floor 1's map certified.
4. **A real lock.** `python3 bringup/relocalise.py --expected-map f5_lobby_20260910` then `--check`.
   Want >= 80% with a wide margin over the runner-up. A 46% "best" with 42% and 40% alternatives
   is ambiguous, not marginal — it will not hold, and one such pose collapsed to 16% within
   seconds of being published.

To put the map back on floor 5: `python3 bringup/floor_swap.py --to 5 --go`. It times out after
30 s waiting for map->odom and reports failure while having ACTUALLY LOADED THE MAP CORRECTLY —
check `saved grid` vs `live /map` in its output, then certify by hand:

```python
import sys; sys.path.insert(0, "."); sys.path.insert(0, "bringup")
import floor_swap
sess = floor_swap.slam_session()
floor_swap.LOADED_MAP.write_text("f5_lobby_20260910 %s\n" % sess)
floor_swap.clear_costmaps()
```

`floor_swap --to 1L` REFUSES: 1L is a task floor with no `car_facing_out` seed role. The mission
does not use floor_swap — it has its own `load_map` + global `find_self` — so this only matters
when swapping by hand.

## How far it got

Best run (`runs/20260911T232832Z_ours_f5_to_f1_elevator`, 30 GB) completed:
call button DOWN -> doors -> into the car -> floor-1 press -> turn -> map swap during the ride.

It then STOPPED at `FIND SELF` on floor 1: the global search never got 3 of 5 searches to agree,
so it refused to drive on an unverified pose. That is correct behaviour, and it is the next thing
to fix. Two open questions the operator had not yet answered:

* did the lift actually reach floor 1?
* was the robot still inside the car when the search ran? Localizing from inside a car with the
  doors just opened is the hardest case; floor 1 locked at 74-83% earlier the same day, but only
  once the robot was out in the lobby.

## Button selection — both failures and both fixes

Neither wrong press was a reach or calibration fault. The arm made contact correctly both times.

**Call plate.** UP and DOWN are the same round blue button 52 px apart. Both candidates were
29x30 px at x=619 and the language ranking took the UPPER (0.446) over the lower (0.296), so the
robot called the lift going up. No spatial pick was running. Fixed by `call_index_from_bottom: 1`
on floor 5. Verified: `SPATIAL PICK: 1 from the bottom of a 2-button column -> center=(670,687)`.

**Car panel.** Reads 6/5/4/3/2/1 top to bottom; the robot pressed 5. The frame measured mean
brightness 30.8/255 (the call plate measures 120.6) and only the top two buttons were detected,
plus duplicate boxes of them at three scales. The picker counted BOXES, reported a confident
"4-button column", took the bottom, and got 5.

Fixed in `bringup/detect_frame.py` by `dedupe()` (IoU alone missed nested duplicates — a 32x39
and an 18x25 box on one button have IoU 0.36 — so containment over the smaller box is checked
too) and `pick_from_column()`, which takes the panel's button count from `floors.yaml` and
REFUSES to index a column shorter than that, dividing the panel strip instead. Verified on the
next run: `column held only 3 of 6 buttons, so the panel strip 63x212 px was divided into 6:
button 1 from the bottom is at (458,687)`.

The darkness is the ROOT cause and is NOT fixed. `rgb_camera.auto_exposure_priority` was False,
which pins the driver to 30 fps and caps exposure so auto-exposure cannot brighten the car. It
was set true live but never persisted, and could not be validated in a lit lobby (112.9 -> 114.3).
The strip division works regardless of lighting, which is why it was done that way.

`select_button_count: 6` is read off a photo. If that lift has a B or anything below "1", the
count and therefore the index are wrong. CONFIRM BEFORE TRUSTING.

## Recording

`--bag full` is `ros2 bag record -a`, which subscribes VOLATILE. Against a TRANSIENT_LOCAL
publisher that is compatible, so nothing warns — but the latched message is never delivered.
`/map` recorded 0 on every run until a QoS override file was added (now automatic in
`run_dataset.py`). Verified 0 -> 2.

`/ouster/points_clean` and `/scan_filtered` recorded 0 whenever the mission's own bring-up
restarted their publishers after recording began. `--ready` skips `load_map` and adopts the live
lock via `adopt_lock`, so nothing is rebuilt underneath the recorder. Verified 0 -> 3256 / 3250.

A complete run looks like: /map 2, tf_static 4, points_clean ~3256, scan_filtered ~3250,
camera ~8300, global costmap ~211, local costmap ~550, /plan ~111, ~944 frames, ~31 events.

## Calibration — TWO profiles, deliberately separate

| target | offset | selected by |
|---|---|---|
| door / hall call plate | `[0.021, -0.0293, 0.025]` | default |
| elevator car panel | `[0.032, -0.0443, 0.025]` | `--offset-profile lift_car_select` |

Do not merge them; they differ by 15 mm in y, more than either was trimmed by. The last 5 mm on
`lift_car_select` was applied against a SYNTHESISED strip-division target, so it corrects strip
geometry as well as the tool model.

Per-target reach settings live in `config/floors.yaml`: call/select/task `_align_mm` and
`_standoff_mm`. The in-car "1" reads the DESTINATION floor's settings, because mission.sh presses
`B_SELECT_QUERY` and B is the destination — `select_standoff_mm` belongs on 1L, not on 5.

Standoff is -200 everywhere. The mission default of -45 stops 155 mm short of where contact
actually happened on all three confirmed presses.

`task_align_mm: -100` on 1L only: the ADA plate at the +45 default folds J2 to -67 deg, past the
-55 limit UFACTORY Studio holds to keep the arm off the laptop on the deck.

## Open problems

1. **The controller picks its own IK branch.** `approach_target.py` commands CARTESIAN poses, so
   `get_inverse_kinematics()` pre-checks describe a solution the arm is under no obligation to
   use. The ADA press ended at J4=+151.8 when the check predicted -28.0. Every press so far
   worked DESPITE this. The fix is commanding joint angles for the reach; it was deliberately not
   done on the eve of a run.
2. **`tool_tip_mm = 172` is still the unverified catalogue figure.** The -200 standoff is doing
   the work it should be doing.
3. **The floor-1 ADA waypoint is ~91 mm too close.** The confirmed press happened after the
   operator nudged the base back; the recorded `f1_lobby_ada_button` was never updated, and the
   chance to re-record it was missed. With `task_align_mm: -100` it no longer hits the J2 limit,
   so it will not refuse — but at the recorded pose the wrist solution flips where it did not at
   the good one. Re-record it when next on floor 1.
4. **`mission.sh`'s press note prints the wrong standoff.** It reads `${UTP_STANDOFF:--45}` rather
   than the per-target value, so it says "driven -45 mm past the target" on a press that used
   -200. Cosmetic, but it lies in the log.
5. **Contact evidence exists.** `ControllerError 31` fires and IS the press succeeding — proven on
   the ADA plate. `mission.sh` already relies on this. The note in `calib/handeye.json` that said
   otherwise has been corrected.
