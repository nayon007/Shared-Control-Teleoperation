# Drawer-Opening Scenario in Gazebo

A simulated Franka Research 3 (FR3) holds the handle of a drawer and must open it by 0.25 m. Any motion off the drawer axis turns into force at the handle. If that off-axis force exceeds 30 N, the handle counts as lost and the trial fails.

This README covers installation, launch, controls and reset. The baseline is direct teleoperation, run by a scripted operator or a DualSense controller.

**Requirements:** Linux with an X11 desktop and Docker. An NVIDIA GPU is expected by the default setup. ROS 2 Humble and Gazebo 11 run inside the container, so they need no host installation. The commands were tested on Ubuntu 20.04.

Each code block says where to run it: `# --- on the host ---` means a normal terminal, and `# --- inside the container ---` means a shell opened with `docker compose exec dev bash`. Lines starting with `#` are only labels, so pasting them does nothing. `<project>` stands for the folder containing this file.

## 1. Installation

Do this once.

**1. Install Docker, the NVIDIA Container Toolkit and the DualSense device rule** (asks for your sudo password):

```bash
# --- on the host (normal terminal) ---
cd <project>
bash scripts/setup_host_docker.sh
```

Log out and back in, then check that Docker can see the GPU:

```bash
# --- on the host (normal terminal) ---
docker run --rm --gpus all ubuntu:22.04 nvidia-smi
```

**2. Make sure the FR3 robot model is present.** `ros2_ws/src/franka_description` must contain files. If it's empty, get it at the pinned version:

```bash
# --- on the host (normal terminal) ---
git clone --depth 1 --branch 2.9.0 https://github.com/frankaemika/franka_description.git ros2_ws/src/franka_description
```

**3. Build the container image.** This downloads about 4 GB.

```bash
# --- on the host (normal terminal) ---
docker compose build
```

**4. Start the container and build the workspace:**

```bash
# --- on the host (normal terminal) ---
docker compose up -d
docker compose exec dev bash
# --- inside the container ---
cd ~/SCE_571_ws/ros2_ws && colcon build --symlink-install
```

The summary should end with `4 packages finished`. The project folder is shared with the container, so `~/SCE_571_ws` inside the container is your `<project>` folder.

## 2. Launch

Do this at the start of each session.

```bash
# --- on the host (normal terminal) ---
cd <project>
xhost +local:docker            # lets the container open the Gazebo window
docker compose up -d
docker compose exec dev bash
# --- inside the container ---
source ~/SCE_571_ws/ros2_ws/install/setup.bash
ros2 launch sc_gazebo sim.launch.py ideal_gravity_comp:=true
```

The Gazebo window opens with the gripper on the drawer handle. Wait until the terminal prints:

```
[sim_startup]: simulation ready
```

This took about 8 s in testing; the very first start can take longer. Leave this terminal running and open a second one for the trial (Section 3):

```bash
# --- on the host (normal terminal) ---
docker compose exec dev bash
# --- inside the container ---
source ~/SCE_571_ws/ros2_ws/install/setup.bash
```

`ideal_gravity_comp:=true` is needed for valid trials. Without it, gravity effects on the arm's joint controller alone can exceed the 30 N limit.

**Useful launch options:**

| Option | Default | Meaning |
|---|---|---|
| `yaw:=2.74159` | `3.14159` | Drawer axis direction in radians. 3.14159 points at the robot; the trials also used 2.74159 and 3.54159 |
| `friction:=3.0` | `3.0` | Drawer friction [N] |
| `damping:=5.0` | `5.0` | Drawer damping [Ns/m] |
| `mass:=2.0` | `2.0` | Drawer mass [kg] |
| `gui:=false` | `true` | Run without the Gazebo window |

## 3. Controls

Each run below is one trial. Reset (Section 4) before starting the next one.

### Scripted operator

The script pushes at 0.1 m/s along the drawer axis, rotated by a fixed heading error.

```bash
# --- inside the container ---
ros2 run sc_gazebo baseline_direct.py --heading 0     # should succeed
ros2 run sc_gazebo baseline_direct.py --heading 2     # should lose the handle
```

To save the results, add `--out trial.json --log trial.csv`.

### DualSense controller

Connect the controller by USB. In one container terminal start the controller driver, and in another start the trial:

```bash
# --- inside the container ---
ros2 run joy joy_node
ros2 run sc_gazebo baseline_direct.py --joy --timeout 60
```

| Control | Action |
|---|---|
| **R2 trigger** | Speed and deadman: released = no motion, fully pressed = 0.1 m/s. Press it once to activate it |
| Left stick, left / right | Move the hand along the robot base x axis (right = +x) |
| Left stick, up / down | Move the hand along the base y axis (up = +y) |
| Right stick, up / down | Move the hand up and down (up = +z) |

With the default scene, **hold the left stick left and squeeze R2** to open the drawer.

The controller path has been tested only with simulated stick messages, not on a physical DualSense. To try it without a controller, run this instead of `joy_node`:

```bash
# --- inside the container ---
python3 ~/SCE_571_ws/scripts/fake_joy.py --lx 1 --r2 -1 --seconds 10
```

The stick axis numbers assume a Linux DualSense through `joy_node`. If a real controller moves the wrong way, check them with `ros2 topic echo /joy`.

### When a trial ends

The trial stops by itself and prints its results as one line of JSON:

| `status` | Meaning |
|---|---|
| `success` | The drawer opened 0.25 m with off-axis force below 30 N |
| `handle_lost` | Off-axis force went over 30 N (`final_opening_m` shows how far it got) |
| `timeout` | Time limit reached: 15 s by default, change it with `--timeout` |
| `ik_failed` | The commanded position is out of the arm's reach |

Expected results on the default scene:

| Heading error | Status | Opening reached |
|---|---|---|
| 0° | `success` | 0.25 m in about 2.6 s |
| 2° | `handle_lost` | about 0.11 m |

## 4. Reset

Reset before every trial. It stops the simulation and starts it again with the drawer closed, and takes about 11 s.

In a container terminal, stop the simulation:

```bash
# --- inside the container ---
bash ~/SCE_571_ws/scripts/stop_sim.sh
```

Then run the launch command again and wait for `simulation ready`:

```bash
# --- inside the container ---
ros2 launch sc_gazebo sim.launch.py ideal_gravity_comp:=true
```

Or do both steps from the host, without the Gazebo window:

```bash
# --- on the host (normal terminal) ---
bash scripts/run_sim_bg.sh gui:=false ideal_gravity_comp:=true
```

Always stop the old simulation first. Leftover processes can make the next launch fail or load the wrong scene.

To finish a session:

```bash
# --- inside the container ---
bash ~/SCE_571_ws/scripts/stop_sim.sh
# --- on the host (normal terminal) ---
docker compose down
```

## 5. Repeat the experiments

To rerun all 48 trials (about 20 minutes, with the container running):

```bash
# --- on the host (normal terminal) ---
bash scripts/run_pilot.sh
python3 scripts/analyze_pilot.py
```

Results go to `results/phase3/`. Trials already saved in `results/phase3/trials/` are skipped, so move that folder away to run everything again. `analyze_pilot.py` needs numpy and matplotlib on the host.

## 6. Troubleshooting

| Problem | Fix |
|---|---|
| No Gazebo window, or a display error | Run `xhost +local:docker` on the host, then launch again |
| `permission denied` from Docker | Log out and back in after the installation |
| `ros2: command not found`, or the package isn't found | Run `source ~/SCE_571_ws/ros2_ws/install/setup.bash`, after building the workspace |
| `ros2` commands fail with `!rclpy.ok()` | Run `ros2 daemon stop` |
| The script waits and nothing happens | The simulation isn't ready: wait for `simulation ready` in the launch terminal |
| Spawn timeout on the first start | Run `stop_sim.sh` and launch again |
| A trial fails at 0° heading error | Check that `ideal_gravity_comp:=true` was used, and that the simulation was reset first |
