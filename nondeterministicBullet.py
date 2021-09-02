import pybullet as p
import pybullet_data

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())


def setupWorld(velocity):
    global kukaId
    p.resetSimulation()
    p.setPhysicsEngineParameter(deterministicOverlappingPairs=1)
    p.setGravity(0, 0, -10)
    p.loadURDF("plane.urdf")
    kukaId = p.loadURDF("kuka_iiwa/model_free_base.urdf",
                        [0, 0, 0], useFixedBase=True)
    p.loadURDF("kuka_iiwa/model_free_base.urdf",
               [0.5, 0, 0], useFixedBase=True)
    for i in range(p.getNumJoints(kukaId)):
        p.setJointMotorControl2(
            kukaId, i, p.VELOCITY_CONTROL, targetVelocity=velocity)
    p.stepSimulation()
    p.setGravity(0, 0, -10)


def simmulate():
    # inconsistency starts at step 488
    for i in range(488):
        p.stepSimulation()
    return p.getJointState(kukaId, 1)


def check_consistency(world_number):
    setupWorld(world_number / 100)
    p.saveState()
    state_a = simmulate()

    p.restoreState(stateId=0)
    state_b = simmulate()
    if state_a == state_b:
        print(f"World {world_number} is deterministic so far")
    else:
        print(f"World {world_number} is non deterministic")


for i in range(29):
    check_consistency(i)
