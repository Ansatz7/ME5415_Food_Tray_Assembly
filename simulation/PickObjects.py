objectFilePath = 'Objects/'

def add_marker(rootNode, i:int, position:list, scale:float, color:list=[1,1,0]):
    # Add sphere to pickup
    marker = rootNode.addChild(f'marker{i}')

    # Solver/time integrators to calculate system positions and velocities at each time ste
    #visualization
    markerVisu = marker.addChild('markerVisu')
    markerVisu.addObject('MeshOBJLoader', name='loader', filename='mesh/sphere.obj')
    markerVisu.addObject('OglModel', name='Visual', src='@loader', translation=position, color=color, scale=scale)


def add_cube(rootNode, position:list, mass:float, scale:float):
    '''Adds rigid cube to scene to pickup'''
    cube = rootNode.addChild('cube')

    # Solver/time integrators to calculate system positions and velocities at each time step
    cube.addObject('EulerImplicitSolver', name='odesolver')
    cube.addObject('SparseLDLSolver', name='linearSolver')
    cube.addObject('MechanicalObject', template='Rigid3', position=position+[0, 0, 0, 1])
    cube.addObject('UniformMass', totalMass=mass)
    cube.addObject('UncoupledConstraintCorrection')

    #collision
    cubeCollis = cube.addChild('cubeCollis')

    cubeCollis.addObject('MeshOBJLoader', name='loader', filename='mesh/smCube27.obj', triangulate=True,  scale=scale+0.1)

    cubeCollis.addObject('MeshTopology', src='@loader')
    cubeCollis.addObject('MechanicalObject')
    cubeCollis.addObject('TriangleCollisionModel')
    cubeCollis.addObject('LineCollisionModel')
    cubeCollis.addObject('PointCollisionModel')
    cubeCollis.addObject('RigidMapping') # Maps DOFs of cube mesh to a rigid SOFA object (use other types of mapping for soft objects)

    #visualization
    cubeVisu = cube.addChild('cubeVisu')

    cubeVisu.addObject('MeshOBJLoader', name='loader', filename='mesh/smCube27.obj')

    cubeVisu.addObject('OglModel', name='Visual', src='@loader', color=[0.0, 0.1, 0.5], scale=scale) # Visual mesh *slightly* smaller than actual object model/collision mesh
    cubeVisu.addObject('RigidMapping')


def add_fixed_cube(rootNode, position:list, scale:float):
    '''Adds rigid cube to scene to pickup'''
    cube = rootNode.addChild('cube')

    # Solver/time integrators to calculate system positions and velocities at each time step
    cube.addObject('EulerImplicitSolver', name='odesolver')
    cube.addObject('SparseLDLSolver', name='linearSolver')
    cube.addObject('MechanicalObject', template='Rigid3', position=position+[0, 0, 0, 1])
    cube.addObject('UniformMass', totalMass=1.0)
    cube.addObject('UncoupledConstraintCorrection')

    #collision
    cubeCollis = cube.addChild('cubeCollis')

    cubeCollis.addObject('MeshOBJLoader', name='loader', filename='mesh/smCube27.obj', triangulate=True,  scale=scale)

    cubeCollis.addObject('MeshTopology', src='@loader')
    cubeCollis.addObject('MechanicalObject')
    cubeCollis.addObject('TriangleCollisionModel')
    cubeCollis.addObject('LineCollisionModel')
    cubeCollis.addObject('PointCollisionModel')
    cubeCollis.addObject('RigidMapping') # Maps DOFs of cube mesh to a rigid SOFA object (use other types of mapping for soft objects)

    #visualization
    cubeVisu = cube.addChild('cubeVisu')

    cubeVisu.addObject('MeshOBJLoader', name='loader', filename='mesh/smCube27.obj')

    cubeVisu.addObject('OglModel', name='Visual', src='@loader', color=[1.0, 1.0, 0.0], scale=scale+0.1) # Visual mesh *slightly* smaller than actual object model/collision mesh
    cubeVisu.addObject('RigidMapping')

    # Use a bounding box Region of Interest to get the topmost points in the gripper only
    box = [position[0]-50, position[1]-50, 0, position[0]+50, position[1]-50, 105]
    cube.addObject('BoxROI', name='boxROI', box=box, strict=False) 

    # Fix the topmost points of the gripper
    cube.addObject('FixedProjectiveConstraint', name='fixedpoint', indices='0')#indices='@boxROI.indices')



def add_sphere(rootNode, position:list, mass:float, scale:float):
    '''Adds rigid sphere to scene to pickup'''
    sphere = rootNode.addChild('sphere')

    # Solver/time integrators to calculate system positions and velocities at each time step
    sphere.addObject('EulerImplicitSolver', name='odesolver')
    sphere.addObject('SparseLDLSolver', name='linearSolver')
    sphere.addObject('MechanicalObject', template='Rigid3', position=position+[0, 0, 0, 1])
    sphere.addObject('UniformMass', totalMass=mass)
    sphere.addObject('UncoupledConstraintCorrection')

    #collision
    sphereCollis = sphere.addChild('sphereCollis')

    sphereCollis.addObject('MeshOBJLoader', name='loader', filename='mesh/sphere.obj', triangulate=True,  scale=scale-0.1) # Make collision hitbox slightly bigger than visual
    # sphereCollis.addObject('MeshOBJLoader', name='loader', filename='details/data/mesh/Sphere.obj', triangulate=True,  scale=1)
    # sphereCollis.addObject('MeshSTLLoader', name='loader', filename='details/data/mesh/Sphere.stl', triangulate=True,  scale=1)

    sphereCollis.addObject('MeshTopology', src='@loader')
    sphereCollis.addObject('MechanicalObject')
    sphereCollis.addObject('TriangleCollisionModel')
    sphereCollis.addObject('LineCollisionModel')
    sphereCollis.addObject('PointCollisionModel')
    sphereCollis.addObject('RigidMapping') # Maps DOFs of sphere mesh to a rigid SOFA object (use other types of mapping for soft objects)

    #visualization
    sphereVisu = sphere.addChild('sphereVisu')

    sphereVisu.addObject('MeshOBJLoader', name='loader', filename='mesh/sphere.obj')
    # sphereVisu.addObject('MeshOBJLoader', name='loader', filename='details/data/mesh/Sphere.obj')
    # sphereVisu.addObject('MeshOBJLoader', name='loader', filename='details/data/mesh/Sphere.stl')

    sphereVisu.addObject('OglModel', name='Visual', src='@loader', color=[0.0, 0.0, 1.0], scale=scale)
    sphereVisu.addObject('RigidMapping')



def add_deformable_object(rootNode, position:list, mass:float, scale:float, 
                          modulus:float, poissonRatio:float, 
                          vtk_path:str, stl_path:str, name:str, colour:list=[1.0, 1.0, 0.0, 1.0], modelPositionCorrection:list=[0,0,0], rotation:list=[0,0,0]):
    '''Adds deformable object to scene to pickup'''
    
    deformableObject = rootNode.addChild(name)
    # Add solvers
    deformableObject.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.1, rayleighMass=0.1)
    deformableObject.addObject('SparseLDLSolver', name='preconditioner', template='CompressedRowSparseMatrixMat3x3d')

    # Load mesh
    deformableObject.addObject('MeshVTKLoader', name='loader', filename=objectFilePath+vtk_path, translation=position, scale=scale, rotation=rotation)
    deformableObject.addObject('MeshTopology', src='@loader', name='container')

    # Add mechanical properties
    deformableObject.addObject('MechanicalObject', name='tetras', template='Vec3d', showIndices=False, showIndicesScale=4e-5)
    deformableObject.addObject('UniformMass', totalMass=mass)
    deformableObject.addObject('BoxROI', name='boxROISubTopo', box=[position[0]-50, position[1]-50, 0, position[0]+50, position[1]+50, 150], strict=False)
    # Describes the internal forces/stresses generated when object is deformed. This particular type corresponds to elastic material deformation w large rotations 
    deformableObject.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=poissonRatio,  youngModulus=modulus)
    deformableObject.addObject('LinearSolverConstraintCorrection', name='ccorrection')
    
    # Add collisions to deformableObject 
    collisions = deformableObject.addChild('collisions')
    collisionPosition = [position[i]+modelPositionCorrection[i] for i in range(len(position))]
    collisions.addObject('MeshSTLLoader', name='loader', filename=objectFilePath+stl_path, translation=collisionPosition, scale=scale, rotation=rotation)
    
    collisions.addObject('MeshTopology', src='@loader', name='topo') # Creates a topology using the mesh loaded by STL loader above
    collisions.addObject('MechanicalObject', name='collisMech')
    collisions.addObject('TriangleCollisionModel', selfCollision=False) # Collision using the mesh defined in mesh topology
    collisions.addObject('LineCollisionModel', selfCollision=False) # Self collision turned off, since deformableObject is (unlikely) to collide with itself
    collisions.addObject('PointCollisionModel', selfCollision=False)
    collisions.addObject('BarycentricMapping') # Maps collision mesh to deformableObject 

    # Add a visual object to visualise deformableObject
    visualisation = deformableObject.addChild('visu')
    visualisation.addObject('MeshVTKLoader', name='loader', filename=objectFilePath+vtk_path, translation=position, scale=scale, rotation=rotation)
    visualisation.addObject('OglModel', src='@loader', color=colour)
    visualisation.addObject('BarycentricMapping')
    return deformableObject

def add_brocolli(rootNode, position:list, mass:float):
    '''Adds deformable brocolli to scene to pickup'''

    youngModulusBrocolli = 30 # 30 MPa
    poissonRatioBrocolli = 0.3

    add_deformable_object(rootNode, position, mass, scale=1.0, modulus=youngModulusBrocolli, poissonRatio=poissonRatioBrocolli, 
                          vtk_path='broccoli.vtk', stl_path='broccoli.stl', colour=[0, 1, 0, 1], name='broccoli', modelPositionCorrection=[-25, -25, 0])

def add_sausage(rootNode, position:list, mass:float):
    '''Adds deformable sausage to scene to pickup'''

    youngModulusSausage = 1.85 # 1.85 MPa
    poissonRatioSausage = 0.49

    add_deformable_object(rootNode, position, mass, scale=0.5, modulus=youngModulusSausage, poissonRatio=poissonRatioSausage, 
                          vtk_path='sausage.vtk', stl_path='sausage.stl', colour=[1.0, 0, 0, 1], name='sausage')
    

def add_meatball(rootNode, position:list, mass:float, scale:float=1.0):
    '''Adds deformable meatball to scene to pickup. scale controls size (1.0 = original).'''

    youngModulusMeatball = 1.4 # 1.4 MPa
    poissonRatioMeatball = 0.49

    add_deformable_object(rootNode, position, mass, scale=scale, modulus=youngModulusMeatball, poissonRatio=poissonRatioMeatball,
                          vtk_path='meatball.vtk', stl_path='meatball.stl', colour=[1.0, 0, 0, 1], name='meatball', modelPositionCorrection=[-13, 0, -11])
    

def add_carrot(rootNode, position:list, mass:float):
    '''Adds deformable carrot to scene to pickup'''

    youngsModulus = 3.0 # 7 MPa
    poissonRatio = 0.4

    add_deformable_object(rootNode, position, mass, scale=0.6, modulus=youngsModulus, poissonRatio=poissonRatio, 
                          vtk_path='carrot.vtk', stl_path='carrot.stl', colour=[0.9, 0.6, 0, 1], name='carrot', modelPositionCorrection=[-15, -15,0]) 
    
def add_green_beans(rootNode, position:list, mass:float):
    '''Adds deformable bean to scene to pickup'''

    youngsModulus = 9 # 9 MPa
    poissonRatio = 0.4

    add_deformable_object(rootNode, position, mass, scale=0.15, modulus=youngsModulus, poissonRatio=poissonRatio, 
                        vtk_path='bean.vtk', stl_path='bean.stl', colour=[0, 1.0, 0, 1], name='bean') 

    # for y in range(0, 6, 5):
    #     for x in range(0, 6, 5):
    #         bean_position = [position[0]+x, position[1]+y, position[2]]
    #         add_deformable_object(rootNode, bean_position, mass, scale=0.15, modulus=youngsModulus, poissonRatio=poissonRatio, 
    #                             vtk_path='bean.vtk', stl_path='bean.stl', colour=[0, 1.0, 0, 1], name='bean') 

    # bean_position = [position[0]+5.0, position[1], position[2]+5.0]
    # add_deformable_object(rootNode, bean_position, mass, scale=1.0, modulus=youngsModulus, poissonRatio=poissonRatio, 
    #                     vtk_path='bean.vtk', stl_path='bean.stl', colour=[1.0, 0, 0, 1], name='bean') 

    # bean_position = [position[0], position[1]+5.0, position[2]+10.0]
    # add_deformable_object(rootNode, bean_position, mass, scale=1.0, modulus=youngsModulus, poissonRatio=poissonRatio, 
    #                     vtk_path='bean.vtk', stl_path='bean.stl', colour=[1.0, 0, 0, 1], name='bean') 


def add_cookie(rootNode, position:list, mass:float):
    '''Adds deformable cookie to scene to pickup'''

    youngsModulus = 180.0 # 80 MPa
    poissonRatio = 0.4

    add_deformable_object(rootNode, position, mass, scale=0.6, modulus=youngsModulus, poissonRatio=poissonRatio, 
                          vtk_path='cookie.vtk', stl_path='cookie.stl', colour=[1, 0.8, .2, 1], name='cookie', modelPositionCorrection=[0,-23,0]) 

def add_spaghetti(rootNode, position:list, mass:float):
    '''Adds deformable spaghetti to scene to pickup'''

    youngsModulus = 1.3 # 1.3 MPa
    poissonRatio = 0.4

    add_deformable_object(rootNode, position, mass, scale=3.0, modulus=youngsModulus, poissonRatio=poissonRatio, 
                          vtk_path='spaghetti.vtk', stl_path='spaghetti.stl', colour=[1.0, 1.0, 0.3, 1], name='spaghetti', modelPositionCorrection=[-2,0,-2]) 

    # newPosition = [position[0]+3, position[1], position[2]]             
    # add_deformable_object(rootNode, newPosition, mass, scale=1.0, modulus=youngsModulus, poissonRatio=poissonRatio, 
    #                       vtk_path='spaghetti.vtk', stl_path='spaghetti.stl', colour=[1.0, 1.0, 0.3, 1], name='spaghetti') 

    # newPosition = [position[0]+7, position[1], position[2]]    
    # add_deformable_object(rootNode, newPosition, mass, scale=1.0, modulus=youngsModulus, poissonRatio=poissonRatio, 
    #                       vtk_path='spaghetti.vtk', stl_path='spaghetti.stl', colour=[1.0, 1.0, 0.3, 1], name='spaghetti') 

def add_orangeJuice(rootNode, position:list, mass:float):
    '''Adds deformable orangeJuice to scene to pickup'''

    youngsModulus = 2000.0 # 2000 MPa
    poissonRatio = 0.4

    add_deformable_object(rootNode, position, mass, scale=0.5, modulus=youngsModulus, poissonRatio=poissonRatio, 
                          vtk_path='cup.vtk', stl_path='cup.stl', colour=[1, 0.8, .2, 1], name='orangeJuice', modelPositionCorrection=[-23,24,0], rotation=[90,0,0]) 
    

def add_eggs(rootNode, position:list, mass:float):
    '''Adds deformable eggs to scene to pickup'''

    youngsModulus = 0.045 # 45 KPa
    poissonRatio = 0.4

    add_deformable_object(rootNode, position, mass, scale=1.0, modulus=youngsModulus, poissonRatio=poissonRatio, 
                          vtk_path='eggs.vtk', stl_path='eggs.stl', colour=[1, 1.0, 0.0, 1], name='eggs', modelPositionCorrection=[-12,0,0]) 