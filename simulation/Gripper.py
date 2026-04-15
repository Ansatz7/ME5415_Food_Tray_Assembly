import math
from PickObjects import add_marker

youngModulusFingers = 590        # MPa — silicone-like material
youngModulusStiffLayerFingers = 5500  # MPa — stiffer inextensible layer

scale = 1e3
radius = 30   # mm, radial distance from gripper centre to finger base
zHeight = 100

cadFilePath = 'CAD/'

# Vertex indices of the finger base/mount end (local Y < 5 mm in finger.vtk).
# Found by reading the ASCII VTK: all vertices with second coordinate < 5.
# Using these directly avoids unreliable BoxROI world-space computations.
_BASE_INDICES = [
    8, 9, 10, 11, 12, 21,
    50, 51, 52, 53, 54, 55, 71, 72, 73, 74,
    163, 167, 169, 171, 172, 173, 174, 175, 176, 177, 178,
    210, 211, 217, 243, 247, 249,
]


def _compute_finger_layout(numFingers: int):
    """
    Compute translations and rotation angles for `numFingers` fingers
    evenly spaced around the gripper axis (z-axis).
    Returns (translations, angles) where each entry corresponds to one finger.
    """
    angles = [i * 2 * math.pi / numFingers for i in range(numFingers)]
    translations = []
    for a in angles:
        x = radius * math.sin(a)
        y = radius * math.cos(a)
        translations.append(f'{x} {y} {zHeight}')
    return translations, angles


def add_gripper(rootNode, numGrippers: int = 4):
    """
    Add a pneumatic soft gripper to the scene.

    Parameters
    ----------
    rootNode    : SOFA root node
    numGrippers : number of fingers (default 4 — improved over the 3-finger reference)
    """
    translations, angles = _compute_finger_layout(numGrippers)

    gripper = rootNode.addChild('gripper')

    for i in range(numGrippers):
        a = angles[i]

        # Rz = 270° - a  →  makes the cavity/fingernail face INWARD (toward gripper centre).
        # Derivation: fingernail is local +X; after Rx(-90°)+Rz, it points world (cos Rz, sin Rz).
        # Inward direction from position (sin a, cos a) is (-sin a, -cos a).
        # Solving: cos Rz = -sin a, sin Rz = -cos a  →  Rz = 270° - a.
        rotation = [-90, 0, 270 - a * 180 / math.pi]

        finger = gripper.addChild(f'finger{i+1}')
        # Add solvers — Mat3x3d template is ~3x faster than the default scalar template for 3-DOF nodes
        finger.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.1, rayleighMass=0.1)
        finger.addObject('SparseLDLSolver', name='preconditioner', template='CompressedRowSparseMatrixMat3x3d')

        # Load mesh
        finger.addObject('MeshVTKLoader', name='loader', filename=cadFilePath+'finger.vtk', rotation=rotation, translation = translations[i])
        finger.addObject('MeshTopology', src='@loader', name='container')

        # Add mechanical properties
        finger.addObject('MechanicalObject', name='tetras', template='Vec3d', showIndices=False, showIndicesScale=4e-5)
        finger.addObject('UniformMass', totalMass= 0.0001)
        # Describes the internal forces/stresses generated when object is deformed. This particular type corresponds to elastic material deformation w large rotations
        finger.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=0.3,  youngModulus=youngModulusFingers)

        # Fix the finger base using stiff springs on hardcoded indices (_BASE_INDICES).
        # These 33 vertices are the mount end of finger.vtk (local Y < 5 mm).
        # Using RestShapeSpringsForceField instead of FixedConstraint+BoxROI because:
        #   - BoxROI world-space box must be recomputed per rotation (fragile)
        #   - RestShapeSpring is numerically robust and allows gripper movement
        #     by updating MechanicalObject.rest_position at runtime.
        finger.addObject('RestShapeSpringsForceField',
                         name='baseSpring',
                         points=_BASE_INDICES,
                         stiffness=1e8)

        finger.addObject('LinearSolverConstraintCorrection', name='ccorrection')

        # Constitutuve law of stiff layer
        modelSubTopo = finger.addChild('modelSubTopo')
        if i == 0:
            modelSubTopo.addObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra='@boxROISubTopo.tetrahedraInROI', name='container')
        else:
            modelSubTopo.addObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra='@../../finger1/boxROISubTopo.tetrahedraInROI', name='container')
        # Forcefield for stiffer layer deformation — l_topology explicitly links to the
        # TetrahedronSetTopologyContainer in this node, preventing it from walking up
        # to the parent's MeshTopology (which would cause "must have tetrahedric topology" error)
        modelSubTopo.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                               youngModulus=str(youngModulusStiffLayerFingers-youngModulusFingers),
                               topology='@container')



        # Pneumatic chamber/cavity 
        cavity = finger.addChild(f'cavity')
        # cavity.addObject('MeshSTLLoader', name='loader', filename=cadFilePath+f'cavity.stl',translation = translations[i], rotation=rotation)
        cavity.addObject('MeshVTKLoader', name='loader', filename=cadFilePath+f'cavitywhole.vtk',translation = translations[i], rotation=rotation)
        cavity.addObject('MeshTopology', src='@loader', name='topo')
        cavity.addObject('MechanicalObject', name='cavity') # Has to be mechancial object so it has DOFs/can be deformed

        # This is the heart of the pneumatic actuator; it directly imparts the pressure force onto the mesh container (pneunetCavity) which can be configured
        # Can either define this by pressure or volume growth
        cavity.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=0.0, triangles='@topo.triangles', valueType='pressure')
        cavity.addObject('BarycentricMapping', name='mapping',  mapForces=False, mapMasses=False) # Maps the deformation of the cavity mesh to finger 

        # Add collisions to finger 
        collisionFinger = finger.addChild('collisionFinger')

        if i < 2:
            collisionFinger.addObject('MeshSTLLoader', name='loader', filename=cadFilePath+'finger_hooked.stl', translation = translations[i], rotation=rotation)
        else:
            collisionFinger.addObject('MeshSTLLoader', name='loader', filename=cadFilePath+'finger_hooked.stl', translation = translations[i], rotation=rotation)
        
        collisionFinger.addObject('MeshTopology', src='@loader', name='topo')
        collisionFinger.addObject('MechanicalObject', name='collisMech')
        collisionFinger.addObject('TriangleCollisionModel', selfCollision=False)
        # Line/PointCollisionModel removed — large performance cost, not needed for food grasping
        collisionFinger.addObject('BarycentricMapping')

        # Add a visual object to visualise finger
        modelVisu = finger.addChild('visu')
        # modelVisu.addObject('MeshSTLLoader', name='loader', filename=cadFilePath+'finger.stl')
        modelVisu.addObject('MeshVTKLoader', name='loader', filename=cadFilePath+'finger.vtk', rotation=rotation, translation = translations[i])
        modelVisu.addObject('OglModel', src='@loader', color=[0.4, 0.4, 0.4, 0.6])#color=[0.7, 0.7, 0.7, 0.6])
        modelVisu.addObject('BarycentricMapping')


        # #  Add a visual object to visualise fingernail with different colour
        modelVisu = finger.addChild('fingernail') #fingernail.addChild('visu')
        if i < 2:
            modelVisu.addObject('MeshVTKLoader', name='loader', filename=cadFilePath+'fingernail_hooked.vtk', translation = translations[i], rotation=rotation)
        else:
            modelVisu.addObject('MeshVTKLoader', name='loader', filename=cadFilePath+'fingernail_hooked.vtk', translation = translations[i], rotation=rotation)
        modelVisu.addObject('OglModel', src='@loader', color=[1.0, 0.8, 0.0, 1.0])
        modelVisu.addObject('BarycentricMapping')


