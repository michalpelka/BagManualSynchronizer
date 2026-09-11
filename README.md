# bag_manual_synchronizer
<img width="3773" height="2102" alt="image" src="https://github.com/user-attachments/assets/e6ed38e1-b52b-48cb-9596-691a3d123da6" />

A small C++/OpenGL tool for finding the time offset between two ROS 2 bags by
eye: it overlays their IMU streams on two ImPlot plots, lets you slide one bag
along the time axis until the traces line up, and saves the offset you settled
on.

Bag **A** is the reference and never moves. Bag **B** is shifted, and the value
you save is defined by

```
t_aligned(B) = t(B) + time_offset_seconds
```

## Building

Requires ROS 2 (developed against Jazzy), GLFW, OpenGL and yaml-cpp. Dear ImGui
and ImPlot are downloaded by CMake at configure time.

```bash
source /opt/ros/jazzy/setup.bash

# standalone
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

# or as part of a workspace
colcon build --packages-select bag_manual_synchronizer
```

To build without network access, point CMake at existing checkouts:

```bash
cmake -S . -B build \
  -DIMGUI_SOURCE_DIR=/path/to/imgui \
  -DIMPLOT_SOURCE_DIR=/path/to/implot
```

## Installing

Outside of a colcon workspace, `cmake --install` puts the executable in the
usual place for the chosen prefix (`bin/`, so `/usr/local/bin` by default),
putting it on `PATH`:

```bash
cmake --build build -j
sudo cmake --install build          # or -DCMAKE_INSTALL_PREFIX=~/.local, etc.

bag_manual_synchronizer /path/to/bag_a /path/to/bag_b
```

The same build also installs a copy under `lib/bag_manual_synchronizer/`,
which is the layout `colcon build` / `ros2 run bag_manual_synchronizer
bag_manual_synchronizer` expect; both come from the same `cmake --install`.

Either way, the ROS 2 environment still has to be sourced (`source
/opt/ros/jazzy/setup.bash`) **at run time**, not just at build time: the
executable finds `librclcpp` etc. through an rpath baked in at build time,
but rosbag2's sqlite3/mcap storage plugins are looked up through pluginlib
via `AMENT_PREFIX_PATH`, and that only exists once the environment is
sourced. Without it, both bags will fail to open.

## Running

```bash
./build/bag_manual_synchronizer /path/to/bag_a /path/to/bag_b
./build/bag_manual_synchronizer --config sync_offset.yaml    # resume a session
```

Both bag arguments are optional; paths can also be typed into the UI. A bag is
either a directory containing `metadata.yaml` or a single storage file.

## Using it

1. **Load.** Drag a bag onto the window: the first drop fills bag A, the second
   fills bag B, and further drops cycle back round. The header of the panel that
   will receive the next drop is marked `<< next drop`, and the *drop A next* /
   *drop B next* buttons override the order. Dropping several paths at once
   fills the slots in the order they arrive. You can drop the bag directory or
   its `metadata.yaml`; either resolves to the same bag.

   Paths can also be typed in and confirmed with *Scan*. Either way, every
   `sensor_msgs/msg/Imu` topic in the bag appears in the dropdown; a dropped bag
   selects the first one and loads it straight away, and you can pick a
   different topic and press *Load*. Reading runs on a background thread, so
   the UI stays live.
2. **Pick a clock.** *timestamps* selects whether sample times come from the
   message header stamp (the sensor clock) or from the bag receive time (the
   recording machine, including transport delay). Changing it reloads both bags.
3. **Get roughly aligned.** If the two recordings come from machines with very
   different clocks, *Align bag starts* puts both at the same start instant.
4. **Align precisely.** Drag the offset slider, type an exact value, use the
   nudge buttons, or press Left/Right (Shift for the coarse step). The plots
   update immediately; the alignment is right when the two traces sit on top of
   each other. Zoom with the scroll wheel — both plots share one x range.
5. **Optionally let it guess.** *Estimate offset* runs a normalized
   cross-correlation over the current search radius, starting from the current
   offset, and reports the peak correlation. A low peak means the motion was too
   weak to align on — treat the result as a starting point, not an answer.
6. **Save.** Set an output path and press *Save offset*.
7. **Or bake it in.** *Export MCAP* writes a copy of bag B as an MCAP bag with
   the offset already applied, so downstream tools need to know nothing about
   this tool's output file. The destination defaults to the source bag path
   with `_synced` appended and can be edited; an existing destination is
   refused rather than overwritten. Export runs on a background thread with a
   progress bar and a *Cancel* button.

Both plots (angular velocity and linear acceleration) show both bags at once,
bag A in cool colours and bag B in warm ones. Per-axis checkboxes switch between
the x/y/z components and the vector magnitude; the magnitude is usually the
easiest signal to align on because it is orientation-independent.

Traces are subsampled to *max points/line* for drawing. Turn *decimate* off once
you are zoomed in on the feature you are aligning to, so that every sample is
drawn.

## Output

### The offset file

```yaml
# bag_manual_synchronizer result
# Bag B is aligned to bag A's clock by:
#   t_aligned = t_b + time_offset_seconds
bag_a:
  uri: /data/bag_a
  imu_topic: /imu/data
bag_b:
  uri: /data/bag_b
  imu_topic: /sensor/imu
timestamp_source: header_stamp
time_offset_seconds: -0.734
time_offset_nanoseconds: -734000000
```

Reload it with `--config` or the *Load config* button to pick up where you left
off.

### The exported bag

*Export MCAP* copies every topic of bag B — not just the IMU — into a new bag,
shifting each message by the offset. Two clocks are involved and both are moved:

- the bag's own `recv_timestamp` and `send_timestamp`, always;
- the `stamp` inside the message header, when *shift header stamps too* is on
  (the default).

The second one matters more than it looks. Most consumers — `message_filters`,
TF, rviz — read the header stamp, not the bag timestamp, so an export that
moved only the bag timestamps would still behave as though nothing had been
synchronized.

Header stamps are rewritten only for types whose first field is a
`std_msgs/Header`, which is checked per topic through the introspection type
support. For those, the eight stamp bytes are patched directly in the CDR
buffer, so every other field of every message is copied through bit-for-bit.
Topics of other types are copied unchanged and reported in the status line.

A header stamp of exactly zero means "unset" in ROS rather than 1970, so those
are left at zero instead of being turned into a fabricated time. Stamps that
would go negative are clamped to zero and counted.

## Notes and limitations

- Only a constant offset is estimated. Clock *drift* (a rate difference between
  the two recordings) shows up as traces that align at one end of the bag and
  separate at the other, and this tool cannot correct it.
- Messages with an unset header stamp are skipped, and out-of-order stamps are
  sorted; both are counted and reported under the bag panel.
- Export rewrites timestamps only. Messages that embed a time somewhere other
  than a leading header (a `Path` of stamped poses, say, or a stamp in a custom
  field) keep their original values.
- The whole IMU topic is held in memory: roughly 100 bytes per sample, so an
  hour of 200 Hz IMU is about 70 MB per bag.
