# ME5415 Food Tray Assembly — Soft Gripper Simulation

NUS ME5415 Advanced Soft Robotics, AY2025/26  
Simulation of a cable-driven bellows soft gripper for food tray preparation.  
Built on **SOFA Framework v23.06** + **SoftRobots plugin**.

---

## Project Overview

This project simulates a 5-finger cable-driven soft gripper picking up food items of varying size, texture, and weight, as required by the ME5415 Manipulation Challenge scenario.

Key design decisions vs. the reference project ([yonx30/ME5415_Food_Gripper](https://github.com/yonx30/ME5415_Food_Gripper)):

| Feature | Reference | This project |
|---------|-----------|--------------|
| Finger count | 3 | **5** |
| Finger geometry | PneuNet (pneumatic chambers) | **SensorFinger (cable-driven bellows)** |
| Actuation | SurfacePressureConstraint | **CableConstraint** |
| Base anchoring | FixedConstraint + BoxROI | **RestShapeSpringsForceField** (moveable) |
| Gripper movement | Plane moves, gripper static | **Full gripper translation (XYZ)** |
| Collision plane | Grid mesh (holes) | **Flat invisible quad + decorative grid** |
| Collision models | Triangle only | **Triangle + Line + Point** (anti-穿模) |

The SensorFinger design uses a bellows-style geometry with a dorsal cable tendon. Pulling the cable shortens the outer surface and curls the finger tip inward, producing a compliant power grasp suitable for delicate food objects.

Finger mesh: `simulation/CAD/finger_sensor.vtk` / `finger_sensor.stl` (original SensorFinger geometry).

---

## Quick Start

```bash
# 1. Set up environment (run once per terminal session)
source setup_env.sh

# 2. Run simulation
cd simulation
runSofa -g qglviewer Main.py
```

Requires SOFA v23.06 with SoftRobots plugin. See `setup_env.sh` for paths.

---

## Controls

> **Note:** In the qglviewer GUI, plain arrow keys move the camera. Use **Ctrl + arrow** to send movement commands to the gripper controller.

| Key | Action |
|-----|--------|
| `Ctrl + ↑ / ↓` | Move gripper forward / backward (y-axis) |
| `Ctrl + ← / →` | Move gripper left / right (x-axis) |
| `Ctrl + U / J` | Move gripper up / down (z-axis) |
| `Space` | Pull cable — close gripper |
| `-` (minus) | Release cable — open gripper |

---

## Food Objects Supported

Uncomment the relevant line in `simulation/Main.py` to add to scene.  
All spawn positions assume the virtual platform at **z_col = 60 mm** — set spawn z ≥ 70.

| Object | Function | Mass | Notes |
|--------|----------|------|-------|
| Meatball | `add_meatball` | 5 g | Default demo object |
| Broccoli | `add_brocolli` | 20 g | Soft, deformable |
| Cookie | `add_cookie` | 10 g | Rigid-ish, flat |
| Sausage | `add_sausage` | 70 g | Elongated |
| Green Beans | `add_green_beans` | 0.013 g | Thin, small |
| Fried Eggs | `add_eggs` | 5 g | Very soft (45 KPa) |
| Orange Juice Cup | `add_orangeJuice` | 300 g | Heavy, rigid cup |
| Spaghetti | `add_spaghetti` | 1 g | Thin, scale=3.0 |
| Carrot | `add_carrot` | 100 g | Medium stiffness |

**Verified grasping:** meatball, broccoli, cookie, sausage, green beans, fried eggs, orange juice cup.

---

## Configuration

Key parameters in `simulation/Main.py`:

```python
NUM_FINGERS       = 5      # number of fingers
FINGER_RADIUS     = 60     # mm, radial distance from centre to finger base
INFLATE_INCREMENT = 0.5    # mm of cable pull per Space keypress
MOVE_INCREMENT    = 0.5    # mm per keypress
PRESSURE_LIMITS   = (0.0, 30.0)  # cable displacement range (mm)
```

Solver / collision parameters:

```python
GenericConstraintSolver  maxIterations = 50     # increase to ~200 for very soft objects
LocalMinDistance         alarmDistance = 8      # mm, collision detection range
                         contactDistance = 2    # mm, contact activation threshold
```

Key parameters in `simulation/Gripper.py`:

```python
YoungsModulus  = 3000   # MPa, finger stiffness (lower = softer, better contact)
zHeight        = 170    # mm, finger base height above world origin
                        # finger tip rests at zHeight − 135 ≈ 35 mm
```

Virtual platform height (`simulation/Main.py`):

```python
z_col = 60   # mm, collision plane height (food lands here)
             # visual grid translated to z=50 (grid top face ≈ z_col)
```

---

## Collision Group Scheme

| Object | Group | Collides with |
|--------|-------|---------------|
| Fingers | 1 | Group 0 (food) only |
| Ground plane | 2 | Group 0 (food) only |
| Food objects | 0 (default) | All groups |

This allows fingers to pass through the floor freely while food is still supported.

---

## Repository Structure

```
simulation/
  Main.py                   # SOFA scene entry point
  Gripper.py                # 5-finger SensorFinger layout and FEM setup
  GripperController.py      # Keyboard controller (movement + cable actuation)
  PickObjects.py            # Food object definitions
  SensorFinger/             # Finger parametric design (Gmsh + SOFA source)
    generate_meshes.py      # Run to regenerate VTK/STL meshes
    Generation.py           # Gmsh geometry functions
    Config.py               # Default finger parameters
  CAD/
    finger_sensor.vtk       # FEM volumetric mesh (used by SOFA)
    finger_sensor.stl       # Surface mesh (collision + visual)
    square_grid.stl         # Decorative ground plane visual
    intermediate/           # Intermediate CAD files (not used in simulation)
  Objects/                  # Food object mesh files (VTK/STL)
docs/                       # Assignment brief and rule book
refs/                       # Reference projects (senior project, SoftRobots)
setup_env.sh                # Environment setup script
```

---

## Regenerating Finger Meshes

The simulation uses `finger_sensor.vtk` / `finger_sensor.stl` directly (original SensorFinger geometry). If you want to regenerate or modify the finger shape:

```bash
cd simulation
python3 SensorFinger/generate_meshes.py
```

Output: `CAD/finger_sensor.vtk`, `CAD/finger_sensor.stl`

---

## References

- [SofaDefrost/SoftRobots](https://github.com/SofaDefrost/SoftRobots)
- [SofaDefrost/SoftRobots.DesignOptimization](https://github.com/SofaDefrost/SoftRobots.DesignOptimization)
- [yonx30/ME5415_Food_Gripper](https://github.com/yonx30/ME5415_Food_Gripper) (reference)
