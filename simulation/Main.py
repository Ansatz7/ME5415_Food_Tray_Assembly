"""
ME5415 Advanced Soft Robotics - Food Tray Assembly
NUS, AY2025/26

Simulation of a cable-driven bellows soft gripper (SensorFinger) for food handling.
Built on SOFA Framework v23.06 + SoftRobots plugin.

Modifications from reference (yonx30/ME5415_Food_Gripper):
  - 5-finger gripper layout (vs 3-finger) for improved stability on flat foods
  - SensorFinger (bellows + cable) design instead of PneuNet (pneumatic chambers)
  - Cable actuation via CableConstraint (vs SurfacePressureConstraint)
  - Camera angle adjusted for better overhead view

Controls:
  ↑ / ↓               : Move gripper forward / backward (y-axis)
  ← / →               : Move gripper left / right       (x-axis)
  U / J                : Move gripper up / down          (z-axis)
  Space                : Pull cable (close gripper)
  Minus (-)            : Release cable (open gripper)
"""

import Sofa
import Sofa.Gui
from GripperController import WholeGripperController
from PickObjects import *
from Gripper import add_gripper

cadFilePath = 'CAD/'

# ── Gripper configuration ────────────────────────────────────────────────────
NUM_FINGERS = 2          # 5-finger layout (reference used 3)
FINGER_RADIUS = 50       # mm, radial distance from centre to finger base
PRESSURE_LIMITS = (0.0, 20.0)   # cable displacement limits (mm)
INFLATE_INCREMENT = 1.0  # mm of cable pull per keypress
MOVE_INCREMENT = 1.0
# ─────────────────────────────────────────────────────────────────────────────


def add_plugins(rootNode):
    """Load all required SOFA component plugins."""
    pluginNode = rootNode.addChild('PluginNode')
    pluginNode.addObject('RequiredPlugin', name='SoftRobots')
    for plugin in [
        "Sofa.Component.AnimationLoop",
        "Sofa.Component.Collision.Detection.Algorithm",
        "Sofa.Component.Collision.Detection.Intersection",
        "Sofa.Component.Collision.Geometry",
        "Sofa.Component.Collision.Response.Contact",
        "Sofa.Component.Constraint.Lagrangian.Correction",
        "Sofa.Component.Constraint.Lagrangian.Solver",
        "Sofa.Component.Constraint.Projective",
        "Sofa.Component.Engine.Select",
        "Sofa.Component.IO.Mesh",
        "Sofa.Component.LinearSolver.Direct",
        "Sofa.Component.LinearSolver.Iterative",
        "Sofa.Component.Mapping.Linear",
        "Sofa.Component.Mapping.NonLinear",
        "Sofa.Component.Mass",
        "Sofa.Component.ODESolver.Backward",
        "Sofa.Component.Setting",
        "Sofa.Component.SolidMechanics.FEM.Elastic",
        "Sofa.Component.SolidMechanics.Spring",
        "Sofa.Component.StateContainer",
        "Sofa.Component.Topology.Container.Constant",
        "Sofa.Component.Topology.Container.Dynamic",
        "Sofa.Component.Visual",
        "Sofa.GL.Component.Rendering3D",
        "Sofa.GL.Component.Shader",
    ]:
        pluginNode.addObject('RequiredPlugin', name=plugin, printLog=False)
    return pluginNode


def add_pipelines(rootNode):
    """Set up the simulation and collision pipeline."""
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('CompositingVisualLoop')
    rootNode.addObject('GenericConstraintSolver', name='GCS',
                       tolerance=1e-9, maxIterations=1000,
                       computeConstraintForces=True)
    rootNode.addObject('CollisionPipeline')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('CollisionResponse',
                       response='FrictionContactConstraint',
                       responseParams='mu=1.0')
    rootNode.addObject('LocalMinDistance', name='Proximity',
                       alarmDistance=10, contactDistance=1, angleCone=0.0)
    rootNode.addObject('BackgroundSetting', color='1 1 1', listening='1')
    rootNode.addObject('OglSceneFrame', style='CubesCones', alignment='TopRight')


def add_plane(rootNode, grid_scale=1.2):
    """Add a static ground plane with collision.
    Collision uses a flat quad (no holes) to prevent objects falling through grid gaps.
    Visual uses the decorative grid mesh.
    grid_scale : scale of the visual grid (1.0 = original, 2.0 = double size)
    """
    # ── Collision: invisible flat quad, no holes ──────────────────────────────
    half = 200   # mm half-size of collision quad
    z_col = -2   # raised to match visual grid top surface
    flat_verts = [[-half, -half, z_col], [half, -half, z_col],
                  [half,  half, z_col],  [-half, half, z_col]]
    flat_tris  = [[0, 1, 2], [0, 2, 3]]

    planeNode = rootNode.addChild('Plane')
    planeNode.addObject('MeshTopology', name='topo',
                        position=flat_verts, triangles=flat_tris)
    planeNode.addObject('MechanicalObject', src='@topo')
    planeNode.addObject('TriangleCollisionModel', simulated=False, moving=False)

    # ── Visual: decorative grid mesh ──────────────────────────────────────────
    planeVisu = planeNode.addChild('visu')
    planeVisu.addObject('MeshSTLLoader', name='loader',
                        filename=cadFilePath + 'square_grid.stl',
                        triangulate=True, scale=grid_scale, translation=[0, 0, -10])
    planeVisu.addObject('OglModel', src='@loader', color=[0.5, 0.5, 0.5, 1])
    return rootNode


def add_camera(rootNode, position):
    """Set up the interactive camera."""
    rootNode.addObject('InteractiveCamera', name='camera',
                       position=f'{position[0]} {position[1]} {position[2]}',
                       lookAt=f'{position[0]} 0 0',
                       distance=f'{position[0]} {position[1]} {position[2]}')


def add_gripper_disc(rootNode, disc_r=65, inner_r=10, z=100, thickness=10, N=48):
    """
    Add a thick visual annular disc (mount plate) at height z.
    Has top face, bottom face, outer wall, and inner wall — solid ring appearance.
    disc_r    : outer radius (mm)
    inner_r   : inner hole radius (mm)
    z         : bottom face height (mm) — matches zHeight in Gripper.py
    thickness : disc height (mm)
    N         : polygon resolution
    """
    import math
    zb, zt = z, z + thickness   # bottom and top z

    def ring(r, zz):
        return [[r * math.cos(2 * math.pi * k / N),
                 r * math.sin(2 * math.pi * k / N), zz] for k in range(N)]

    # Vertex layout: 4 rings × N vertices each
    # 0      ..   N-1 : bottom outer
    # N      .. 2N-1  : bottom inner
    # 2N     .. 3N-1  : top outer
    # 3N     .. 4N-1  : top inner
    verts = ring(disc_r, zb) + ring(inner_r, zb) + ring(disc_r, zt) + ring(inner_r, zt)
    B_O, B_I, T_O, T_I = 0, N, 2*N, 3*N

    tris = []
    for k in range(N):
        n = (k + 1) % N
        # Bottom face (winding reversed so normal faces down)
        tris += [[B_O+k, B_I+k, B_O+n], [B_I+k, B_I+n, B_O+n]]
        # Top face
        tris += [[T_O+k, T_O+n, T_I+k], [T_I+k, T_O+n, T_I+n]]
        # Outer wall
        tris += [[B_O+k, B_O+n, T_O+k], [B_O+n, T_O+n, T_O+k]]
        # Inner wall
        tris += [[B_I+k, T_I+k, B_I+n], [B_I+n, T_I+k, T_I+n]]

    disc = rootNode.addChild('gripperDisc')
    disc.addObject('MeshTopology', name='topo', position=verts, triangles=tris)
    disc.addObject('MechanicalObject', src='@topo', name='dofs', template='Vec3d')

    discVisu = disc.addChild('visu')
    discVisu.addObject('OglModel', name='ogl', src='@../topo',
                       color=[0.30, 0.35, 0.55, 0.92])
    discVisu.addObject('IdentityMapping', input='@../dofs', output='@ogl')
    return disc


def createScene(rootNode):
    add_plugins(rootNode)
    add_pipelines(rootNode)
    add_plane(rootNode)
    add_camera(rootNode, [20, -300, 150])
    # ── Gripper ──────────────────────────────────────────────────────────────
    add_gripper(rootNode, NUM_FINGERS, FINGER_RADIUS)
    # add_gripper_disc(rootNode)   # visual mount plate connecting finger bases

    # ── Food objects (uncomment to add to scene) ──────────────────────────────
    # Roundish objects — good for 4-finger wrap grasp
    # add_meatball(rootNode,    [ 0 -15, 20], 0.01)    # 20 g — default demo

    add_meatball(rootNode, [0, -20, 20], 0.005, scale=1.1)
    # add_brocolli(rootNode,  [0,  0,  20], 0.02)    # 20 g
    # add_cookie(rootNode,    [-20,   0,  5], 0.010)   # 10 g

    # Elongated objects — demonstrate lateral compliance
    # add_sausage(rootNode,   [-55, -72, 10], 0.007)  # 70 g
    # add_carrot(rootNode,    [20, -100,  5], 0.1)    # 100 g
    # add_green_beans(rootNode, [15,  0,  5], 0.0000132)

    # Delicate / unusual objects
    # add_spaghetti(rootNode, [15,  0,  1], 0.001)    # 1 g
    # add_eggs(rootNode,      [-16, -16, 0.05], 0.005)  # 5 g
    # add_orangeJuice(rootNode, [0, 0,  0], 0.300)   # 300 g (cup + juice)
    # ─────────────────────────────────────────────────────────────────────────

    controller = WholeGripperController(
        name="controller", node=rootNode,
        pressureLimits=PRESSURE_LIMITS,
        inflateIncrement=INFLATE_INCREMENT,
        moveIncrement=MOVE_INCREMENT,
        numGrippers=NUM_FINGERS,
    )
    rootNode.addObject(controller)

    rootNode.addObject('VisualStyle', displayFlags=(
        'showVisualModels hideBehaviorModels hideCollisionModels '
        'hideBoundingCollisionModels hideForceFields '
        'showInteractionForceFields hideWireframe'
    ))
    rootNode.findData('gravity').value = [0, 0, -9810]
    return rootNode


def main():
    root = Sofa.Core.Node("root")
    createScene(root)
    Sofa.Simulation.init(root)

    # ── Debug: verify FixedConstraint has valid indices ──────────────────────
    try:
        f1_roi = root.gripper.finger1.boxROI
        n_fixed = len(f1_roi.indices.value)
        print(f"[INFO] BoxROI base indices: {n_fixed}")
        if n_fixed == 0:
            print("[WARNING] BoxROI returned 0 indices — fingers will fall!")
        else:
            print(f"[INFO] Base fixation OK — first 5 indices: {list(f1_roi.indices.value)[:5]}")
    except Exception as e:
        print(f"[WARNING] Could not check BoxROI: {e}")
    # ─────────────────────────────────────────────────────────────────────────

    Sofa.Gui.GUIManager.Init("myscene", "qt")
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1200, 900)
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()


if __name__ == '__main__':
    main()
