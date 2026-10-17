Penn RoboRacer Technical Challenge: Wall Follower

## Results

- **Video (Driver's Eye, unlisted):** https://youtu.be/IRIURxUPwYc
- **Fastest clean lap:** `11.201 but didnt record but saved a screenshot, 11.327 recorded with logs shown in the video;` 

## How it works

The node is `autodrive_devkit/autodrive_roboracer/wall_follow_node.py`. It runs once per LiDAR scan (about 17–19 Hz in this simulator)

1. **Centerline wall following.** For each side, two beams are used: one perpendicular to the car (±90°) and one angled 45° forward of it. From these two ranges, the node computes the wall's angle relative to the car (α) and the perpendicular distance. Then it projects that distance 0.4 m ahead: `D_future = D + L·sin(α)`. The error is half the difference between the left and right projected distances, and a PD controller turns it into a steering angle that holds the middle of the track.
2. **Corner handling and safety.** Every wall edge in the scan (a sudden jump in range) is "inflated" by the car's half-width plus a margin. The node then picks the deepest direction the car can actually fit through (disparity extender) and steers toward it. The steering strength depends on the target:
   - **Full strength** when the opening is close, or more than about 35° to the side (hairpin entry and exit).
   - **Gentler** when the opening is far away and roughly straight ahead, scaled by `gap_ref_dist / depth`. Without this, a far target on the straight made the car weave about ±10° about once a second because of the ~0.15 s actuation delay.

   The final steering is a blend: 30% wall-centering and 70% free-space direction on open track. As a wall gets close ahead (a corner), the free-space direction takes over completely. If a wall comes within 0.30 m at the side, the car is pushed away from it.
3. **Speed planner.** The target speed is the lowest of three limits:
   - **v_max:** the straight-line speed cap.
   - **Braking limit,** `sqrt(2·a_brake·free_distance)`: the free distance is measured along the direction the wheels point.
   - **Cornering limit,** `sqrt(a_lat / curvature)`: curvature comes from the steering angle.
4. **Throttle.** Feed-forward plus a PI loop on the measured speed from `/autodrive/roboracer_1/odom`. A negative throttle is used for braking.
5. **Watchdog.** If the LiDAR stream stops for 0.5 s, the car is commanded to stop. Lap completions and collisions are printed to the terminal.

## How it was tuned

1. **Synthetic tracks.** I first tested the controller offline in a small 2D simulator (kinematic bicycle model, ray-cast 1080-beam LiDAR) on synthetic tracks 1.4–2.4 m wide.
2. **Recording.** The node has a recorder (`--ros-args -p record:=true`) that saves every scan, the odometry pose and speed, and the commands. I recorded ten laps in AutoDRIVE.
3. **Track map and car model.** From that recording:
   - **Track map:** I rebuilt the practice track by projecting the scans from the recorded poses. It is a 1.4 m-wide loop with two 180° hairpins and two chevrons.
   - **Car model:** I fitted a model of the AutoDRIVE car's response. Commands take effect about 0.1 s late, the steering servo moves at about 2.5 rad/s, the car understeers, acceleration is about 6 m/s² and braking about 8 m/s². The scan rate is about 19 Hz.
4. **Validation.** A simulator built from that map and model reproduced the real lap times (14.1–14.4 s in the sim vs. 14.1–14.8 s in AutoDRIVE).
5. **Parameter search.** I searched parameters in that simulator. Each candidate had to complete clean laps in a nominal and three pessimistic versions of the car (more delay, slower steering, weaker brakes, noisier LiDAR). The final settings (`v_max` 8, full braking, `a_brake` 3.0) were about 4% faster than the earlier settings and kept more clearance from the walls.
6. **Fixing from recordings.** Two more recordings showed two problems:
   - **Weaving on the straight.** The controller steered as hard at far-away openings as at corners. I fixed this by scaling the steering with the target's distance.
   - **Hairpin-exit crashes.** That fix then weakened the steering at the hairpin exit. I fixed this by restoring full strength for openings off to the side.

   I checked each fix by replaying the recorded scans through the new code before running it. Lap times went from about 13.8 s to about 12.0 s, with no wall contact.
7. **Auto-tuning on the real simulator.** `autodrive_devkit/tools/autotune.py` is a separate tuning tool. It is not part of the driving node. It hill-climbs the node's parameters on the live AutoDRIVE simulator:
   - For each candidate, it sets the parameters through the ROS parameter service and resets the car with `/autodrive/reset_command`.
   - It skips one warm-up lap, then times four laps.
   - It keeps a change only if every lap is clean and the mean lap time improves, and the improvement holds up in a second confirming run.

   An unattended 6-hour run (400 trials) brought the mean lap from 12.4 s to about 11.6 s. Repeat runs of the final settings gave a median lap of 11.55 s and a best of 11.01 s. The tuned values are specific to this practice track. The driving logic itself is unchanged and uses only the live scan.

