import pybullet as p
import pybullet_data
import math
import time

physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally : register the directory
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD) # enable FEM for deformable things

# load a plane
planeId = p.loadURDF("plane.urdf")
#plane = p.createCollisionShape(p.GEOM_PLANE)
# p.createMultiBody(0, plane)

p.setGravity(0,0,-8)


""" CREATE BONES """

# create a cylinder collision shape
cylinderRadius = 0.1
cylinderHeight = 0.3
gap = 0.2
cylinderColPos = [0,0,cylinderHeight/2]
cylinderColOri = p.getQuaternionFromEuler([0,0,1])
cylinderId = p.createCollisionShape(p.GEOM_CYLINDER,radius = cylinderRadius, height = cylinderHeight)#, collisionFramePosition = cylinderColPos)#, collisionFrameOrientation = cylinderColOri)
# to make it static
# boxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[cylinderRadius, cylinderRadius, cylinderRadius])

# link
linkMasses = []
linkCollisionShapeIndices = []
linkVisualShapeIndices = []
linkPositions = []
linkOrientations = []
linkInertialFramePositions = []
linkInertialFrameOrientations = []
linkParentIndices = []
linkJointTypes = []
linkJointAxis = []

baseMass = 1 # 0 : static
basePosition = [0,0,2]#cylinderRadius]
baseOrientation = [1,0,0,1] # rotate 2*theta from a vector (x, y, z). w = cos(theta)
# baseOrientation = p.getQuaternionFromEuler([0, math.pi/2, 0]) # Euler angles

for i in range(9):
    linkMasses.append(baseMass)
    linkCollisionShapeIndices.append(cylinderId)
    linkVisualShapeIndices.append(-1)
    linkPositions.append([0, 0, cylinderHeight + gap])
    linkOrientations.append([0,0,0,1])
    linkInertialFramePositions.append([0,0,0])
    linkInertialFrameOrientations.append([0,0,0,1])
    linkParentIndices.append(i)
    linkJointTypes.append(p.JOINT_SPHERICAL)
    linkJointAxis.append([0,0,1])

# create rigid objects by shapes
snakeId = p.createMultiBody(baseMass, cylinderId, -1, basePosition, baseOrientation, 
                                linkMasses=linkMasses,
                              linkCollisionShapeIndices=linkCollisionShapeIndices,
                              linkVisualShapeIndices=linkVisualShapeIndices,
                              linkPositions=linkPositions,
                              linkOrientations=linkOrientations,
                              linkInertialFramePositions=linkInertialFramePositions,
                              linkInertialFrameOrientations=linkInertialFrameOrientations,
                              linkParentIndices=linkParentIndices,
                              linkJointTypes=linkJointTypes,
                              linkJointAxis=linkJointAxis)
                              # useMaximalCoordinates=True) # seperate links

            
soft = True
numJoints = p.getNumJoints(snakeId)

""" load soft body """
if soft:
    # Available files : cube.obj, cube2.obj, sphere_smooth.obj, duck.obj 
    # p.loadSoftBody("cube2.obj", basePosition = basePosition , scale = 1, mass = 1., useNeoHookean = 0, useBendingSprings=1,useMassSpring=1, springElasticStiffness=40, springDampingStiffness=.1, springDampingAllDirections = 1, useSelfCollision = 0, frictionCoeff = .5, useFaceContact=1, collisionMargin = 0.04)
    print("Start loading...")
    clothId = p.loadSoftBody("soft_cube_snake.obj", basePosition = [0,-2.5,2], scale = 0.5, mass = 1., useNeoHookean = 0, useBendingSprings=1,useMassSpring=1, springElasticStiffness=40, springDampingStiffness=.1, springDampingAllDirections = 1, useSelfCollision = 0, frictionCoeff = .5, useFaceContact=0)
    print("Loading is completed...")

    # remove collisions
    for i in range(-1, numJoints):
        p.setCollisionFilterGroupMask(snakeId, i, 0, 0)
        p.setCollisionFilterPair(0, snakeId, -1, i, 1)
        p.setCollisionFilterPair(snakeId, clothId,i, -1, 0)

""" debug """
debug = True
if soft and debug:
    data = p.getMeshData(clothId, -1, flags=p.MESH_DATA_SIMULATION_MESH)
    print("--------------")
    print("data=",data)
    print(data[0])
    print(data[1])
    text_uid = []
    for i in range(data[0]):
        pos = data[1][i]
        uid = p.addUserDebugText(str(i+1), pos, textColorRGB=[1,1,1])
        text_uid.append(uid)
      
""" Softbody anchor """
if soft:
    print("numJoints :" + str(numJoints))
    # createSoftBodyAnchor params : "softBodyBodyUniqueId", "nodeIndex", "bodyUniqueId", "linkIndex", "bodyFramePosition", "physicsClientId",
    # vtxNum = 4 # 12
    # i = 0
    # for j in range(nodeNum):
    #     p.createSoftBodyAnchor(clothId, i+j, snakeId, i)
    #     p.createSoftBodyAnchor(clothId, i+j+vtxNum, snakeId, i)
    p.createSoftBodyAnchor(clothId, 1, snakeId, 0)
    p.createSoftBodyAnchor(clothId, 2, snakeId, 0)
    p.createSoftBodyAnchor(clothId, 5, snakeId, 0)
    p.createSoftBodyAnchor(clothId, 6, snakeId, 0)

    p.createSoftBodyAnchor(clothId, 33, snakeId, 1)
    p.createSoftBodyAnchor(clothId, 34, snakeId, 1)
    # p.createSoftBodyAnchor(clothId, 36, snakeId, 1)
    p.createSoftBodyAnchor(clothId, 35, snakeId, 1)

    p.createSoftBodyAnchor(clothId, 25, snakeId, 2)
    p.createSoftBodyAnchor(clothId, 26, snakeId, 2)
    p.createSoftBodyAnchor(clothId, 27, snakeId, 2)
    p.createSoftBodyAnchor(clothId, 28, snakeId, 2)

    p.createSoftBodyAnchor(clothId, 29, snakeId, 3)
    p.createSoftBodyAnchor(clothId, 30, snakeId, 3)
    p.createSoftBodyAnchor(clothId, 31, snakeId, 3)
    p.createSoftBodyAnchor(clothId, 32, snakeId, 3)

    p.createSoftBodyAnchor(clothId, 9, snakeId, 4)
    p.createSoftBodyAnchor(clothId, 10, snakeId, 4)
    p.createSoftBodyAnchor(clothId, 11, snakeId, 4)
    p.createSoftBodyAnchor(clothId, 12, snakeId, 4)

    p.createSoftBodyAnchor(clothId, 17, snakeId, 5)
    p.createSoftBodyAnchor(clothId, 18, snakeId, 5)
    p.createSoftBodyAnchor(clothId, 19, snakeId, 5)
    p.createSoftBodyAnchor(clothId, 20, snakeId, 5)

    p.createSoftBodyAnchor(clothId, 13, snakeId, 6)
    p.createSoftBodyAnchor(clothId, 14, snakeId, 6)
    p.createSoftBodyAnchor(clothId, 15, snakeId, 6)
    p.createSoftBodyAnchor(clothId, 16, snakeId, 6)

    p.createSoftBodyAnchor(clothId, 21, snakeId, 7)
    p.createSoftBodyAnchor(clothId, 22, snakeId, 7)
    p.createSoftBodyAnchor(clothId, 23, snakeId, 7)
    p.createSoftBodyAnchor(clothId, 24, snakeId, 7)

    p.createSoftBodyAnchor(clothId, 3, snakeId, 8)
    p.createSoftBodyAnchor(clothId, 4, snakeId, 8)
    p.createSoftBodyAnchor(clothId, 7, snakeId, 8)
    p.createSoftBodyAnchor(clothId, 8, snakeId, 8)


    # p.createSoftBodyAnchor(clothId, 0, snakeId, 0, [0,0,0])


""" Movement """
# change dynamics
anistropicFriction = [1, 0.01, 0.01] # move to X dir
p.changeDynamics(snakeId, -1, lateralFriction=2, anisotropicFriction=anistropicFriction)

for i in range(p.getNumJoints(snakeId)):
    p.changeDynamics(snakeId, i, lateralFriction=2, anisotropicFriction=anistropicFriction)

# calculate movements
dt = 1./240.
SNAKE_NORMAL_PERIOD = 1.5
m_wavePeriod = SNAKE_NORMAL_PERIOD
m_waveLength = 4
m_waveAmplitude = 0.8
m_waveFront = 0.0
m_segmentLength = gap + cylinderHeight
# forward = 0


""" move """
# p.setRealTimeSimulation(0)
start_time = time.time()
while True:
# for i in range(500):
    scaleStart = 1.0

    # start waves
    if (m_waveFront < m_segmentLength * 4.0):
        scaleStart = m_waveFront / (m_segmentLength * 4.0)

    segment = numJoints - 1

    # we simply move the snake forward.
    # this snake may be going backwards, but who can tell ;)
    for joint in range(numJoints):
        segment = joint # numJoints - 1 - joint
        #map segment to phase
        phase = (m_waveFront - (segment + 1) * m_segmentLength) / m_waveLength
        phase -= math.floor(phase)
        phase *= math.pi * 2.0

        #map phase to curvature
        targetPos = math.sin(phase) * scaleStart * m_waveAmplitude
        
        # set our motor (stepSimulation will process motors)
        p.setJointMotorControlMultiDof(snakeId,
                                joint,
                                p.POSITION_CONTROL,
                                targetPosition = [0,targetPos,0, 1],
                                force = [50])
    
        #wave keeps track of where the wave is in time
        m_waveFront += dt / m_wavePeriod * m_waveLength
	
        # rigid_soft_contact and plane_contact
        # print(i,p.getContactPoints(bodyA = snakeId, bodyB = clothId),p.getContactPoints())
    # p.setRealTimeSimulation(1)

    ''' getContactPoints()
    returns the contact points computed during the most recent call to stepSimulation.
    Note that if you change the state of the simulation after stepSimulation,
    the'getContactPoints' is not updated and potentially invalid.
    '''
    # print(p.getContactPoints(bodyA = snakeId, bodyB = clothId))
    p.stepSimulation()
    # time.sleep(dt)

finish_time = time.time()
print("time : " + str(finish_time - start_time))
# p.disconnect()
