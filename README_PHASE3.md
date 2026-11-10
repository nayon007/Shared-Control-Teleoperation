# Drawer-Opening Scenario in Gazebo

A simulated Franka Research 3 (FR3) robot holds a drawer handle and must pull the drawer open by 0.25 m. If the robot pushes sideways, the drawer resists. Once that sideways force passes 30 N, the trial counts as failed ("handle lost").

The project folder is `~/SCE_571_ws`. ROS 2 and Gazebo run inside a Docker container, so they are not installed on the computer itself.

---

## Quick start (every time)

You need **two terminals**. Code blocks below are marked with the terminal to use; lines starting with `#` are only labels, so pasting them does nothing.

### Terminal 1: start the simulation

Open a normal terminal and run:

```bash
# --- Terminal 1, on the computer ---
xhost +local:docker
cd ~/SCE_571_ws
docker compose up -d
docker compose exec dev bash
```

The prompt changes to `dev@...`, which means you're now **inside the container**. Continue there:

```bash
# --- Terminal 1, inside the container ---
source ~/SCE_571_ws/ros2_ws/install/setup.bash
ros2 launch sc_gazebo sim.launch.py ideal_gravity_comp:=true
```

The Gazebo window opens with the robot holding the drawer handle. **Wait for this line** (about 10 s):

```
[sim_startup]: simulation ready
```

Leave Terminal 1 running.

### Terminal 2: run a trial

Open a second normal terminal:

```bash
# --- Terminal 2, on the computer ---
cd ~/SCE_571_ws
docker compose exec dev bash
```

```bash
# --- Terminal 2, inside the container ---
source ~/SCE_571_ws/ros2_ws/install/setup.bash
ros2 run sc_gazebo baseline_direct.py --heading 0
```

In Gazebo the robot pulls the drawer open. Terminal 2 then prints one line of results, which should contain `"status": "success"`.

### Before the next trial: reset

In **Terminal 1**, press `Ctrl+C`, then run:

```bash
# --- Terminal 1, inside the container ---
bash ~/SCE_571_ws/scripts/stop_sim.sh
ros2 launch sc_gazebo sim.launch.py ideal_gravity_comp:=true
```

Wait for `simulation ready` again, then run the next trial in Terminal 2.

### When you're finished

```bash
# --- Terminal 1, inside the container ---
bash ~/SCE_571_ws/scripts/stop_sim.sh
exit
```

```bash
# --- on the computer ---
cd ~/SCE_571_ws
docker compose down
```

---

## Trials to try

Run these in **Terminal 2**, with a reset (above) before each one.

| Command | What happens |
|---|---|
| `ros2 run sc_gazebo baseline_direct.py --heading 0` | Pulls straight: the drawer opens 0.25 m in about 2.6 s, **success** |
| `ros2 run sc_gazebo baseline_direct.py --heading 0.5` | 0.5° off: borderline; on the straight drawer it reaches about 0.22 m before the handle is lost |
| `ros2 run sc_gazebo baseline_direct.py --heading 2` | 2° off: the handle is lost at about 0.11 m, **failure** |
| `ros2 run sc_gazebo baseline_direct.py --heading 2 --out t.json --log t.csv` | Same trial, and also saves the results and the force over time |

The heading is how far, in degrees, the robot's push points away from the drawer's direction. Even 1° is enough to lose the handle.

**Reading the result line:**

| `status` | Meaning |
|---|---|
| `success` | Drawer opened 0.25 m with sideways force under 30 N |
| `handle_lost` | Sideways force went over 30 N; `final_opening_m` shows how far it got |
| `timeout` | Not finished within 15 s |
| `ik_failed` | The robot was asked to reach a position it can't |

**Different drawer direction:** add `yaw:=2.74159` (or `yaw:=3.54159`) to the launch command in Terminal 1. `3.14159`, the default, points straight at the robot.

---

## Drive it yourself with the keyboard (no controller needed)

You need **three terminals**, all inside the container. Open each with `cd ~/SCE_571_ws`, `docker compose exec dev bash`, then `source ~/SCE_571_ws/ros2_ws/install/setup.bash`.

```bash
# --- Terminal 1: the simulation (wait for "simulation ready") ---
ros2 launch sc_gazebo sim.launch.py ideal_gravity_comp:=true
```

```bash
# --- Terminal 2: the trial, listening for controller input for up to 2 minutes ---
ros2 run sc_gazebo baseline_direct.py --joy --timeout 120
```

```bash
# --- Terminal 3: the keyboard controller (keep this terminal selected while driving) ---
python3 ~/SCE_571_ws/scripts/keyboard_joy.py
```

Watch the Gazebo window while you press keys in Terminal 3:

| Key | Does |
|---|---|
| `a` / `d` | Left stick left / right: move the hand along -x / +x |
| `w` / `s` | Left stick up / down: move the hand along +y / -y |
| `i` / `k` | Right stick up / down: move the hand up / down |
| `space` | R2 on / off. **Off = the robot stays still** |
| `1` … `9`, `0` | Speed with R2 on: 10 % … 90 %, `0` = 100 % (0.1 m/s) |
| `x` | Centre both sticks |
| `q` | Quit (releases R2 first) |

Each press moves a stick a quarter of the way, and it **stays there** until you change it, because a terminal can't detect when a key is released. Terminal 3 shows the current stick and R2 state on one line.

**To open the drawer:** press `a` four times (stick fully left), then `space`. The drawer opens, and Terminal 2 prints `"status": "success"` once it reaches 0.25 m. Press `space` at any time to stop the robot.

**To see the problem the project studies:** after `a a a a`, also press `w` once, so the push is slightly sideways. The handle is lost within a few centimetres.

The keyboard controller sends exactly the same `/joy` messages as a DualSense through `joy_node`, so the trial program can't tell them apart. In testing, `a a a a space` opened the drawer in 3 of 3 runs. One earlier run ended with `ik_failed` and could not be reproduced. After every trial, reset as in the Quick start, then start Terminal 2 again.

---

## Controls (DualSense controller)

Connect the controller by USB. Keep the simulation running in Terminal 1, and open two more terminals inside the container (`cd ~/SCE_571_ws` then `docker compose exec dev bash`, then `source ~/SCE_571_ws/ros2_ws/install/setup.bash`):

```bash
# --- Terminal 2, inside the container: controller driver ---
ros2 run joy joy_node
```

```bash
# --- Terminal 3, inside the container: the trial ---
ros2 run sc_gazebo baseline_direct.py --joy --timeout 60
```

| Control | Action |
|---|---|
| **R2 trigger** | Speed and safety switch: released = robot stays still, fully pressed = full speed (0.1 m/s) |
| Left stick left / right | Move the hand along the robot's x axis (right = +x) |
| Left stick up / down | Move the hand along the robot's y axis (up = +y) |
| Right stick up / down | Move the hand up / down |

**To open the drawer: hold the left stick to the left and squeeze R2.**

The controller mode has been tested only with simulated input (the keyboard controller above, and `fake_joy.py`), not with a physical DualSense. `fake_joy.py` holds one fixed input instead of letting you steer; run it in Terminal 2 instead of `joy_node`:

```bash
# --- Terminal 2, inside the container ---
python3 ~/SCE_571_ws/scripts/fake_joy.py --lx 1 --r2 -1 --seconds 10
```

If a real controller moves the wrong way, check the stick values with `ros2 topic echo /joy`.

---

## If something goes wrong

| Problem | Fix |
|---|---|
| `Command 'host$' not found` (or similar) | Copy only the command, not any label in front of it |
| No Gazebo window appears | Run `xhost +local:docker` on the computer (not in the container), then launch again |
| `ros2: command not found` | You're not inside the container, or you skipped `source ~/SCE_571_ws/ros2_ws/install/setup.bash` |
| The trial command waits and nothing happens | The simulation isn't ready yet: wait for `simulation ready` in Terminal 1 |
| `ros2` commands fail with `!rclpy.ok()` | Run `ros2 daemon stop`, then try again |
| A spawn timeout, or the wrong scene appears | Run `bash ~/SCE_571_ws/scripts/stop_sim.sh`, then launch again |
| A 0° trial fails | Check that the launch used `ideal_gravity_comp:=true` and that you reset first |
| `permission denied` from Docker | Log out and back in (needed once after installation) |
| `no configuration file provided` | You're in the wrong folder: run `cd ~/SCE_571_ws` first |

---

## One-time installation

Needed only on a new computer. On this laptop it's already done.

**1. Install Docker, NVIDIA GPU support and the controller permissions** (asks for your password), then log out and back in:

```bash
# --- on the computer ---
cd ~/SCE_571_ws
bash scripts/setup_host_docker.sh
```

**2. Check that the robot model folder `ros2_ws/src/franka_description` isn't empty.** If it is, download it:

```bash
# --- on the computer ---
cd ~/SCE_571_ws
git clone --depth 1 --branch 2.9.0 https://github.com/frankaemika/franka_description.git ros2_ws/src/franka_description
```

**3. Build the container** (downloads about 4 GB) **and the workspace:**

```bash
# --- on the computer ---
cd ~/SCE_571_ws
docker compose build
docker compose up -d
docker compose exec dev bash
```

```bash
# --- inside the container ---
cd ~/SCE_571_ws/ros2_ws
colcon build --symlink-install
```

The build should end with `4 packages finished`. Rebuild this way whenever the code changes.

**Requirements:** Linux with an X11 desktop, Docker, and an NVIDIA GPU. Tested on Ubuntu 20.04.

---

## Repeat all 48 experiments

Takes about 20 minutes. The container must be running (`docker compose up -d`), but the simulation must not be: these scripts start and reset it themselves.

```bash
# --- on the computer ---
cd ~/SCE_571_ws
bash scripts/run_pilot.sh
python3 scripts/analyze_pilot.py
```

Results go to `results/phase3/`. Trials already saved in `results/phase3/trials/` are skipped, so move that folder elsewhere to run all 48 again.
