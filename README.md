# Four-panel Miura robot

A working Python / MuJoCo prototype using your three STL files, with **four motors per panel, 16 independently addressable drives, and the confirmed 1:1 motor-to-wheel ratio**. The original meshes are preserved in `assets/`. Generated meshes separate the three-gear trains and original rotating coupling profiles from each panel frame.

## Run on this Mac

Open Terminal and run:

```bash
cd ~/miura-robot
how
```

The window starts paused. Press **P** to run physics, then use the controls below. You can also double-click `Launch-Simulation.command` in Finder. Mouse drag orbits the camera and scroll zooms; close the window to finish.

| Key/control | Action |
|---|---|
| Up / Down | Unfold with the motors, then request forward / reverse drive |
| Left / Right | Unfold with the motors, then request left / right turning |
| C / O | Stop the drive request and fold / unfold using motor torque |
| Space | Brake drive wheels and hold the current fold; in manual mode, command zero motor speeds |
| X | Coast all motors with zero commanded torque |
| P | Pause / resume physics |
| Right-hand **Control** sliders | Individual motor target speeds, **−40 to +40 rad/s**; switches to manual mode |
| M | Enter manual mode with all motor targets zero |
| [ / ] and + / − | Select a motor and adjust its target by 1 rad/s; other targets continue |
| R / F | Reset to folded / flat inspection pose and pause; these are pose resets, not actuation |

**Tap an arrow to start; tap Space to stop.** Releasing an arrow does not stop motion: MuJoCo's passive-viewer callback supplies key presses but no key-release events. Arrows follow panel 0's heading, not the camera. Defaults are a 0.04 m/s forward request and a 0.4 rad/s turning request; these are wheel-mixer inputs, not guaranteed chassis speeds.

Normal driving and folding use only the 16 motor actuators. No body forces, pose assignments or powered hinge actuators produce motion. The eight motors along shared creases drive their wheels **and** the original interlocking connectors. Their speeds are mechanically constrained by the fold. The other eight drive the perimeter wheels. Folding lifts those perimeter wheels away from the ground, so arrow driving first requests a flat pose. Shared wheels resist rolling; this prototype creeps and slips, and cannot drive freely at a fixed folded shape with the confirmed rigid 1:1 coupling.

The former native sliders exposed raw actuator controls while Python supplied its own torque commands; they were not a 0–1 motor-speed limit. The new viewer has a separate display model whose **Control** sliders really request motor rad/s. Zero requests braking, negative reverses direction, and 1 rad/s is about 9.55 rpm. The overlay shows requested and actual speed. Changing a shared-edge motor's slider can affect the whole fold, and conflicting targets can stall against the linkage. Native joint-position sliders and mouse perturbations do not alter the physical simulation.

**Simulation torque assumption:** `motor_torque_limit_nm` is now **0.08 N·m**, up from the earlier 0.015 N·m placeholder. The earlier value stalled during lifting with the assumed panel masses. At 0.08 N·m, the motor controller folds and unfolds under gravity, with finite tracking error. This is a simulation assumption, not a measured specification or confirmation that your real motors can lift the robot.

Run a headless sequence through all 16 motors and save telemetry:

```bash
.venv/bin/python -m miura_robot.run --demo --seconds 20
.venv/bin/python -m pytest -q
```

Verification: **14 automated tests pass**, including fold → unfold → fold under gravity, forward/reverse/turning direction, slider isolation, individual motor routing and the command watchdog. A separate 20-second sequence through all motors completed with zero MuJoCo warnings and a maximum closure error of 0.053 mm. With the present rigid coupling, a 3-second drive test advances about 2.9 mm or reverses 2.8 mm; turns achieve about ±0.095 rad. These are simulated outcomes with assumed parameters.

The simulation uses seconds, metres, kilograms, radians and newton-metres. CSV results go to `output/telemetry.csv`; a summary is written beside it. `output/robot-preview.png` is an actual MuJoCo render of the initial assembly. A sequential motor test verifies actuation; it is not a designed locomotion gait.

For a fresh installation (Python 3.10 or newer):

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[dev]'
.venv/bin/python -m miura_robot.build
```

On macOS, use `mjpython` for the interactive viewer. Linux/Windows can use `python -m miura_robot.run --viewer`. Headless physics needs no display.

## Drive multiple motors together

```bash
.venv/bin/mjpython -m miura_robot.run --viewer --motors p0_m0=8 p0_m1=8 p1_m2=-4 --seconds 60
```

Each `NAME=RAD_S` entry sets a motor target in the same control cycle. Use the sliders or `[` / `]` and `+` / `−` to edit any motor while the others continue. **Space brakes all, X coasts, and P pauses physics.** All 16 motors remain independently addressable, but the eight shared-edge motor speeds cannot be independently realized because the shafts are connected to the fold.

## Coordinated driving and folding in Python

```python
from miura_robot import Simulation
from miura_robot.motion import RobotMotion

robot = Simulation()
motion = RobotMotion(robot)
try:
    motion.set_fold(1.0)             # Close using the motor shafts
    for _ in range(600):
        robot.command(motion.targets(dt=0.01))
        state = robot.step(0.01)
    motion.drive(forward_m_s=0.04)   # Unfold, then drive; negative reverses
    for _ in range(1000):
        robot.command(motion.targets(dt=0.01))
        state = robot.step(0.01)
    motion.stop()                   # Brake and hold; keep refreshing targets
finally:
    robot.stop()                   # Zero torque / coast
```

`motion.drive(yaw_rad_s=0.4)` requests a left turn. `motion.set_fold(0.0)` opens the panels. `targets(dt)` advances the fold controller's integral feedback by the control interval; send the returned batch and step physics for that same interval. High-level pose/contact feedback currently comes from `Simulation`; a real controller will need encoder and orientation feedback.

Headless examples:

```bash
.venv/bin/python -m miura_robot.run --drive --fold 1.0 --seconds 6
.venv/bin/python -m miura_robot.run --drive --forward 0.04 --seconds 10
.venv/bin/python -m miura_robot.run --drive --turn -0.4 --seconds 10
```

## Control individual motors from Python

Run in the project directory, or install the package with the command above:

```python
from miura_robot import Simulation, MotorCommand

robot = Simulation()
print(robot.names)  # p0_m0 ... p3_m3
try:
    for _ in range(1000):
        robot.command({
            "p0_m0": MotorCommand("velocity", 8.0),  # motor rad/s
            "p1_m2": MotorCommand("velocity", -4.0),
            "p3_m3": MotorCommand("torque", 0.002),  # motor N m
        })
        state = robot.step(0.01)
        # state contains all encoder angles, speeds, commanded torques,
        # three crease angles and fourth-crease closure error.
finally:
    robot.stop()
```

`set_velocity(name, rad_s)` and `set_torque(name, nm)` are convenience methods. Commands for omitted motors persist for at most 0.25 seconds since their last update, then become zero torque. Refresh commands every control cycle. `stop()` clears all drive torque and coasts; it does not brake or freeze the robot. Velocity commands use a torque-limited proportional controller. Reported torque is the commanded motor torque, not a sensor measurement. Commands are validated before a batch is applied, and values outside configured limits raise an exception.

Names use panel index 0–3 and drive index 0–3. Panel colors: 0 cyan, 1 orange, 2 green, 3 purple. In the **original STL XY view**, drive indices are: 0 upper slanted edge, 1 left vertical edge, 2 right vertical edge, 3 lower slanted edge. Signs follow the shaft axes recorded in `models/manifest.json`; a positive command does not mean the same world direction on every folded panel.

## Mechanical model and assumptions

Confirmed by you: four panels, four motors on each panel, 1:1 transmission, the supplied motors fitting beside the gears, and each output shaft driving both its wheel and its mating interlock. Motor specifications are not yet available.

The STL envelope is approximately 141.802 × 205.160 × 12 source units; source units are **assumed to be millimetres**. The wheel is approximately 16 mm in diameter and 4 mm thick. STL contains geometry, not assembly constraints, material data or a motor datasheet.

The panels now connect **directly through their original interlocking profiles**. There are no generated gray brackets, rods, sleeves or spacers. The incorrect reflection of panel 2 has been corrected, so all panels face the same side when flat and complementary profiles line up. The nominal facet edge is 150 mm, with **zero separation between mating axes**. The projecting connector envelopes overlap by about 12 mm and their solid profiles interleave in the flat pose. The wheel and motor placements relative to each panel are unchanged from the three-gear revision.

Three hinge joints and a fourth-crease closure constraint make a single-vertex Miura cell. The unfolded crease rays are 0°, 60°, 180° and 300°. This gives one internal folding degree of freedom away from the flat singularity. The eight shared-edge output shafts drive these creases through joint constraints. There are no additional fold actuators. The builder separates the original male coupling profiles from the fixed frame and attaches them to the output shafts, so their geometry rotates with the neighboring panel.

Each drive now has **three separate rotating gears**: motor/input gear → intermediate gear → outer/output gear → wheel. The intermediate gear reverses direction, and the output returns to the input direction at the confirmed overall 1:1 ratio. The other two gear crowns were fused into the frame in the STL, so the builder removes those crowns and reconstructs them from the separate gear mesh. Supports and shafts remain in the frame. Ideal joint constraints transmit the motion; tooth contact, backlash and electrical motor dynamics are not simulated.

Motors are now coaxial with the inner gear, seated in the adjacent rectangular slots. Wheel shafts are moved **22 mm outward**, across two 11 mm gear-center spacings, to the outermost gear. The three gear positions and connector mounting points are recorded in `models/manifest.json`. Motor readback reports motor-shaft speed; `wheel_velocity_rad_s` reports the actual output joint speed.

Inter-panel self-collision uses inset frame cores, motor boxes and wheel cylinders. Ground contact uses full frame hulls. The inset avoids filling the original connector recesses with a convex collision hull, which would incorrectly push the mated panels apart. These proxies are conservative for panel cores and do not resolve outer frame detail or interlocking-feature contact.

**The interlocks use ideal hinge motion.** The original profiles fit in the flat pose. During folding, the model permits intersection inside the existing lobed coupling and circular bearing regions; it does not simulate tooth disengagement, elastic deformation or detailed bearing clearance. This is a limitation of the mechanical joint model, not proof of physical folding clearance. Mass, torque, speed and friction also need measured values.

Check actual CAD solid intersections separately from the collision proxies:

```bash
.venv/bin/python -m miura_robot.inspect_geometry
```

This checks all panel pairs at five fold angles (0, 0.3, 0.55, 0.9 and 1.2 rad), including frames, motors, gears and wheels. `output/geometry-check.json` reports total intersection, intersection inside original joint regions, and intersection elsewhere. It fails if overlap **outside the joint regions** exceeds 0.001 mm³. The current five-angle sweep has less than 0.000005 mm³ overlap outside joint regions. Rotating the original male couplers with the shafts reduces joint-region overlap to less than 0.8 mm³ total at each sample; these residual intersections are still reported explicitly.

`output/interlock-closeup.png` shows the original profiles mating, `output/interlocked-flat.png` shows the flat arrangement, and `output/robot-preview.png` shows the folded assembly. `output/panel-drivetrain.png` shows the unchanged motor/gear/wheel placements. Regenerate the images on macOS with `.venv/bin/mjpython -m miura_robot.render` (requires the development dependencies).

## Calibrate and rebuild

Edit `config.json`, then run:

```bash
.venv/bin/python -m miura_robot.build
```

The model uses a snapshot of the configuration in `models/manifest.json`; changing the JSON alone does not change an already built model.

| Parameter | Initial value | Status |
|---|---:|---|
| Motor-to-wheel ratio | 1:1 | Confirmed |
| Torque limit | 0.08 N m | Assumed for lifting tests; real motor rating unknown |
| Speed limit | 40 rad/s | Placeholder |
| Panel-frame mass | 0.12 kg | Placeholder |
| Motor / wheel / gear mass | 0.010 / 0.003 / 0.002 kg | Placeholders |
| Gear efficiency | 0.8 | Placeholder |
| Tire friction | 0.9 | Placeholder |
| Initial crease angle | 0.55 rad | Configurable initial condition |
| Wheel axial placement | 9 mm from outer gear center | Inferred, needs confirmation |
| Mating-axis separation | 0 mm | Direct original-profile engagement |

`cad_calibration` stores the nominal connector vertex/edge length, 11 mm gear-center spacing, motor shaft directions, motor placement and the two lobed interlock stations in source coordinates. The former spacer-gap setting has been removed. Rebuild and rerun the geometry check after any calibration changes. `fixed_base: true` anchors panel 0 for bench tests. Panel inertia and rotor armature are approximate; measured mass properties will improve dynamics.

## Connect to the real robot later

`HardwareBridge` exposes the same `command`, `set_velocity`, `set_torque`, `read`, and `stop` operations. It requires an explicitly provided transport, so installing or running this project never connects to hardware.

Implement the `Transport` protocol in `miura_robot/control.py` for the controller you choose:

```python
class MyController:
    def send(self, commands):
        # Map p0_m0 ... p3_m3 to controller IDs.
        # Translate motor rad/s or N m into the controller's supported units.
        # Transmit one frame, with a sequence number and expiration time.
        ...

    def read(self):
        # Return real encoder telemetry, timestamps and fault status.
        ...

    def stop(self):
        # Disable output on all channels using the actual controller protocol.
        ...
```

This is an interface specification, not a completed driver. A motor STL does not identify its electrical interface. A brushed DC driver typically needs encoder feedback and a speed controller; torque control needs current sensing and a measured torque constant. A smart servo may provide these modes directly. Do not map N m directly to PWM.

After implementing the transport with measured limits, use:

```python
from miura_robot import HardwareBridge, MotorCommand

# controller, motor_names and measured_config are supplied by your application.
with HardwareBridge(controller, motor_names, measured_config) as robot:
    robot.arm()
    # In your periodic control loop:
    robot.command({"p0_m0": MotorCommand("velocity", 1.0)})
    robot.tick()
```

Call `tick()` periodically even when no commands arrive. It expires stale motors independently and sends zero torque for them. The MCU must also enforce its own command watchdog and support a physical disable: a host-side timer cannot run after Python crashes or the cable disconnects. The transport must explicitly reject modes the controller cannot implement. Confirm IDs, encoder signs, gear ratio, torque/current limits and stop behavior with wheels unloaded before ground testing. The hardware adapter has only been tested with a fake transport; no real robot has been connected.

## Source references

The model uses [MuJoCo joint actuators and connect constraints](https://mujoco.readthedocs.io/en/latest/XMLreference.html), and the GUI follows the [Python passive-viewer API and macOS launcher requirement](https://mujoco.readthedocs.io/en/latest/python.html). The solver treats the fourth crease as a soft equality constraint; closure residual is logged so its accuracy can be assessed under motion.
