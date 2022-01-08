import pybullet as p
import pybullet_data
import numpy as np
import math

useGui = True

if (useGui):
  p.connect(p.GUI)
else:
  p.connect(p.DIRECT)
offset = [0,0,0]
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", useFixedBase=True)

p.loadURDF("world/urdf/wall.urdf", [9, 0, 0],useFixedBase=True)
idx1= p.loadURDF("world/urdf/wall.urdf", [-9, 0, 0],useFixedBase=True)
idx=p.loadURDF("world/urdf/wall_yaw.urdf", [0, -9, 0], useFixedBase=True)
p.loadURDF("world/urdf/wall_yaw.urdf", [0, 9, 0], useFixedBase=True)
p.loadURDF("world/urdf/corridor.urdf", [4.5, 4.5, 0],useFixedBase=True)
p.loadURDF("world/urdf/corridor.urdf", [-4.5, 4.5, 0],useFixedBase=True)
p.loadURDF("world/urdf/corridor.urdf", [4.5, -5, 0],useFixedBase=True)
p.loadURDF("world/urdf/corridor.urdf", [-4.5, -5, 0],useFixedBase=True)
p.loadURDF("cube_small.urdf", list(np.random.randint(low=-9, high=9, size=3)), globalScaling = 10)

# boundaries = p.getAABB(idx)
# lwh = np.array(boundaries[1])-np.array(boundaries[0])
# boundaries1 = p.getAABB(idx1)
# lwh1 = np.array(boundaries1[1])-np.array(boundaries1[0])
# print(boundaries)
# print(boundaries1)

turtleId = p.loadURDF("turtlebot.urdf", offset)
numJoints = p.getNumJoints(turtleId)
if (numJoints != 32):
  exit()


p.setGravity(0, 0, -10)

for joint in range(numJoints):
    print(p.getJointInfo(turtleId, joint))

distance = 1000
img_w = 128
img_h = 128
aspect = img_w / img_h
fov = 90

targetVel = 100  # rad/s
maxForce = 100  # Newton

rayFrom = []
rayTo = []
rayIds = []

numRays = 1024

rayLen = 13

rayHitColor = [1, 0, 0]
rayMissColor = [0, 1, 0]

replaceLines = True

for i in range(numRays):
  rayFrom.append([0, 0, 1])
  rayTo.append([
      rayLen * math.sin(2. * math.pi * float(i) / numRays),
      rayLen * math.cos(2. * math.pi * float(i) / numRays), 1
  ])
  if (replaceLines):
    rayIds.append(p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor))
  else:
    rayIds.append(-1)

if (not useGui):
  timingLog = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "rayCastBench.json")

numSteps = 10
if (useGui):
  numSteps = 327680

for i in range(numSteps):
  p.stepSimulation()
  for j in range(8):
    results = p.rayTest(rayFrom, rayTo, j + 1)
    print("result:", results)

  #for i in range (10):
  #	p.removeAllUserDebugItems()

  if (useGui):
    if (not replaceLines):
      p.removeAllUserDebugItems()

    for i in range(numRays):
      hitObjectUid = results[i][0]

      if (hitObjectUid < 0):
        hitPosition = [0, 0, 0]
        p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, replaceItemUniqueId=rayIds[i])
      else:
        hitPosition = results[i][3]
        p.addUserDebugLine(rayFrom[i], hitPosition, rayHitColor, replaceItemUniqueId=rayIds[i])

  #time.sleep(1./240.)

if (not useGui):
  p.stopStateLogging(timingLog)

while True:
    for joint in range(2):
        p.setJointMotorControl(turtleId, joint, p.VELOCITY_CONTROL, targetVel, maxForce)
    

    agent_pos, agent_orn = p.getBasePositionAndOrientation(turtleId)

    yaw = p.getEulerFromQuaternion(agent_orn)[-1]
    xA, yA, zA = agent_pos
    zA = zA + 0.3966 # make the camera a little higher than the robot

    # compute focusing point of the camera
    xB = xA + math.cos(yaw) * distance
    yB = yA + math.sin(yaw) * distance
    zB = zA

    view_matrix = p.computeViewMatrix(
                        cameraEyePosition=[xA, yA, zA],
                        cameraTargetPosition=[xB, yB, zB],
                        cameraUpVector=[0, 0, 1.0]
                    )

    projection_matrix = p.computeProjectionMatrixFOV(
                            fov, aspect, nearVal=0.02, farVal=3.5)

    imgs = p.getCameraImage(img_w, img_h,
                            view_matrix,
                            projection_matrix, shadow=True,
                            renderer=p.ER_BULLET_HARDWARE_OPENGL)
    p.stepSimulation()


# targetVel = 1
# for joint in range(2):
#     p.setJointMotorControl(turtleId, joint, p.VELOCITY_CONTROL, targetVel, maxForce)

# while True:
#     p.setJointMotorControl(turtleId, joint, p.VELOCITY_CONTROL, targetVel, maxForce)
#     p.applyExternalForce(r_cube,-1, list(np.random.randint(low=-20, high=20, size=3)),[0,0,0], p.LINK_FRAME)
#     p.stepSimulation()


