from planning import *
from klampt.model import collide
from klampt.model import ik
from klampt.model.trajectory import Trajectory,RobotTrajectory
from klampt.math import vectorops,so3,se3
from klampt.plan import robotplanning
from klampt import vis
from klampt import RobotModel
import time
import numpy as np


def is_collision_free_grasp(world, robot, object):
    finger_pad_links = ['gripper:Link_4', 'gripper:Link_6']

    if robot.selfCollides():
        print('robot self collide')
        return False
    for i in range(world.numTerrains()):
        for j in range(robot.numLinks()):
            if robot.link(j).geometry().collides(world.terrain(i).geometry()):
                print('robot link', j, 'collides with world terrian', i)
                return False
    for i in range(world.numRigidObjects()):
        for j in range(robot.numLinks()):
            # print(j, robot.link(j).getName())
            if i == object.index and robot.link(j).getName() in finger_pad_links:
                # print('skip object', i, 'and link', j)
                continue
            if robot.link(j).geometry().collides(world.rigidObject(i).geometry()):
                print('robot link', j, 'collides with object', i)
                return False
    return True

def retract(robot,gripper,amount,local=True):
    """Retracts the robot's gripper by a vector `amount`.

    if local=True, amount is given in local coordinates.  Otherwise, its given in
    world coordinates.
    """
    if not isinstance(gripper,(int,str)):
        gripper = gripper.base_link
    link = robot.link(gripper)
    Tcur = link.getTransform()
    if local:
        amount = so3.apply(Tcur[0],amount)
    obj = ik.objective(link,R=Tcur[0],t=vectorops.add(Tcur[1],amount))
    res = ik.solve(obj)
    if not res:
        return None
    return robot.getConfig()

def feasible_plan(world, robot, object, qtarget):
    """Plans for some number of iterations from the robot's current configuration to
    configuration qtarget.  Returns the first path found.

    Returns None if no path was found, otherwise returns the plan.
    """
    t0 = time.time()

    moving_joints = [1, 2, 3, 4, 5, 6, 7]

    kwargs = {'type':'sbl','perturbationRadius':0.5, 'bidirectional': 1, 'shortcut':True,'restart':True,
              'restartTermCond':"{foundSolution:1,maxIters:100}"}
    def check_collision(q):
        robot.setConfig(q)
        return is_collision_free_grasp(world, robot, object)
    planner = robotplanning.planToConfig(world=world, robot=robot, target=qtarget, edgeCheckResolution=0.1,
                                         movingSubset=moving_joints, extraConstraints=[check_collision], **kwargs)
    # qtarget is not feasible
    if planner is None:
        return None
    path = None
    numIters = 0
    increment = 100  # there is a little overhead for each planMore call, so it is best not to set the increment too low
    t0 = time.time()
    while time.time() - t0 < 100:  # max 20 seconds of planning
        planner.planMore(increment)
        numIters = numIters + 1
        path = planner.getPath()
        if path is not None:
            if len(path) > 0:
                print("Solved, path has", len(path), "milestones")
                break

    t1 = time.time()
    print("Planning time,", numIters, "iterations", t1 - t0)

    # to be nice to the C++ module, do this to free up memory
    planner.space.close()
    planner.close()

    return path


def plan_pick_one(world,robot,object,gripper,grasp):
    """
    Plans a picking motion for a given object and a specified grasp.

    Arguments:
        world (WorldModel): the world, containing robot, object, and other items that
            will need to be avoided.
        robot (RobotModel): the robot in its current configuration
        object (RigidObjectModel): the object to pick.
        gripper (GripperInfo): the gripper.
        grasp (Grasp): the desired grasp. See common/grasp.py for more information.

    Returns:
        None or (transit,approach,lift): giving the components of the pick motion.
        Each element is a RobotTrajectory.  (Note: to convert a list of milestones
        to a RobotTrajectory, use RobotTrajectory(robot,milestones=milestones)

    Tip:
        vis.debug(q,world=world) will show a configuration.
    """
    qstart = robot.getConfig()
    qstartopen = gripper.set_finger_config(qstart, gripper.partway_open_config(1))  # open the fingers of the start to match qpregrasp
    robot.setConfig(qstartopen)

    grasp.ik_constraint.robot = robot  #this makes it more convenient to use the ik module

    def feasible():
        return is_collision_free_grasp(world, robot, object)

    solved = ik.solve_global(objectives=grasp.ik_constraint, iters=50, numRestarts=5, activeDofs=[1, 2, 3, 4, 5, 6, 7],feasibilityCheck=feasible)
    print('ik status:', solved)
    if not solved: return None

    qpregrasp = robot.getConfig()
    robot.setConfig(qpregrasp)
    if not feasible(): return None

    qtransit = retract(robot=robot, gripper=gripper, amount=list(-0.1*np.array(gripper.primary_axis)), local=True)
    if qtransit is None: return None
    print(qtransit)
    robot.setConfig(qtransit)
    if not feasible(): return None


    robot.setConfig(qpregrasp)

    qopen = gripper.set_finger_config(qpregrasp, gripper.partway_open_config(1))  # open the fingers further
    robot.setConfig(qopen)
    if not feasible(): return None

    # robot.setConfig(qopen)
    qgrasp = gripper.set_finger_config(qopen, gripper.partway_open_config(0.5)) # close the gripper

    robot.setConfig(qgrasp)
    qlift = retract(robot=robot, gripper=gripper, amount=[0, 0, 0.1], local=False)
    robot.setConfig(qlift)
    if not feasible(): return None

    robot.setConfig(qstart)
    transit = feasible_plan(world, robot, object, qtransit)  # decide whether to use feasible_plan or optimizing_plan
    if not transit:
        return None

    return RobotTrajectory(robot,milestones=[qstart]+transit),RobotTrajectory(robot,milestones=[qtransit, qpregrasp, qopen, qgrasp]),\
           RobotTrajectory(robot,milestones=[qgrasp,qlift])


def plan_pick_grasps(world,robot,object,gripper,grasps):
    """
    Plans a picking motion for a given object and a set of possible grasps, sorted
    in increasing score order.

    Arguments:
        world (WorldModel): the world, containing robot, object, and other items that
            will need to be avoided.
        robot (RobotModel): the robot in its current configuration
        object (RigidObjectModel): the object to pick.
        gripper (GripperInfo): the gripper.
        grasp (Grasp): the desired grasp. See common/grasp.py for more information.

    Returns:
        None or (transit,approach,lift): giving the components of the pick motion.
        Each element is a RobotTrajectory.  (Note: to convert a list of milestones
        to a RobotTrajectory, use RobotTrajectory(robot,milestones=milestones)

    Tip:
        vis.debug(q,world=world) will show a configuration.
    """
    qinit = robot.getConfig()
    # sort grasps with increasing score
    sorted_grasps = sorted(grasps, key=lambda grasp: grasp.score)
    ret_val = None
    for i, grasp in enumerate(sorted_grasps):
        print(i)
        robot.setConfig(qinit)
        ret_val = plan_pick_one(world, robot, object, gripper, grasp)
        # todo: the collision-free grasps are not always successful????
        if ret_val is not None:
            break
    return ret_val

