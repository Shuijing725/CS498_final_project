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
        print('retract ik failed!')
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

def clip_angle(num):
    if num > 2*np.pi:
        num = num - 2*np.pi
    elif num < -2*np.pi:
        num = num + 2*np.pi
    return num

def plan_pick_one(world,robot,object,gripper,grasp, robot0=True):
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
        # return True

    solved = ik.solve_global(objectives=grasp.ik_constraint, iters=50, numRestarts=5, activeDofs=[1, 2, 3, 4, 5, 6, 7],feasibilityCheck=feasible)
    print('ik status:', solved)
    if not solved: return None

    qpregrasp = robot.getConfig()
    robot.setConfig(qpregrasp)
    if not feasible(): return None


    qtransit = retract(robot=robot, gripper=gripper, amount=list(-0.1*np.array(gripper.primary_axis)), local=True)

    secondary_axis_gripper = gripper.secondary_axis
    if not isinstance(gripper,(int,str)):
        gripper1 = gripper.base_link
    else:
        gripper1 = gripper
    link = robot.link(gripper1)
    R_gripper_w, _ = link.getTransform()
    secondary_axis_world = so3.apply(R_gripper_w, secondary_axis_gripper)
    secondary_axis_world_2d = np.array(secondary_axis_world)[:-1]
    angle = np.arccos(np.dot(secondary_axis_world_2d, [0, 1]))
    # angle = np.pi/4
    q_rotate = copy.deepcopy(qtransit)
    # if not robot0:
    #     angle = -angle
    q_rotate[7] = clip_angle(q_rotate[7] - angle)
    qpregrasp[7] = clip_angle(qpregrasp[7] - angle)




    if qtransit is None: return None
    print(qtransit)
    robot.setConfig(qtransit)
    if not feasible(): return None


    robot.setConfig(qpregrasp)

    qopen = gripper.set_finger_config(qpregrasp, gripper.partway_open_config(1))  # open the fingers further
    robot.setConfig(qopen)
    if not feasible(): return None

    # robot.setConfig(qopen)
    if robot0:
        close_amount = 0.1
    else:
        close_amount = 0.8
    qgrasp = gripper.set_finger_config(qopen, gripper.partway_open_config(close_amount)) # close the gripper

    robot.setConfig(qgrasp)

    qlift = retract(robot=robot, gripper=gripper, amount=[0, 0, 0.1], local=False)

    robot.setConfig(qlift)
    if not feasible(): return None

    robot.setConfig(qstart)
    transit = feasible_plan(world, robot, object, qtransit)  # decide whether to use feasible_plan or optimizing_plan
    if not transit:
        return None

    return RobotTrajectory(robot,milestones=[qstart]+transit),\
           RobotTrajectory(robot,milestones=[qtransit, q_rotate, qpregrasp, qopen, qgrasp]),\
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


class StepResult:
    FAIL = 0
    COMPLETE = 1
    CONTINUE = 2
    CHILDREN = 3
    CHILDREN_AND_CONTINUE = 4


class PQNode:
    def __init__(self, key, value):
        self.key = key
        self.value = value

    def __lt__(self, other):
        return self.value < other.value

    def __str__(self):
        return str("{} : {}".format(self.key, self.value))


class MultiStepPlanner:
    """A generic multi step planner that can be subclassed to implement multi-step planning
    behavior.

    Subclass will need to:
        - implement choose_item() or provide a sequence of items to solve to the initializer.
        - implement solve_item() to complete items of the plan
        - implement score() for a custom heuristic (lower scores are better)

    """

    def __init__(self, items=None):
        self.W = [PQNode({}, 0)]
        if items is not None:
            self.items = items
        else:
            self.items = []
        self.pending_solutions = []

    def choose_item(self, plan):
        """Returns an item that is not yet complete in plan.  Default loops through items
        provided in the constructor (fixed order), but a subclass can do more sophisticated
        reasoning.

        Args:
            plan (dict): a partial plan.
        """
        for i in self.items:
            if i not in plan:
                return i
        return None

    def on_solve(self, plan, item, soln):
        """Callback that can do logging, etc."""
        pass

    def solve_item(self, plan, item):
        """Attempts to solve the given item in the plan.  Can tell the high-level planner to
        stop this line of reasoning (FAIL), the solution(s) completes the plan (COMPLETE), this item
        needs more time to solve (CONTINUE), several options have been generated and we wish
        stop planning this item (CHILDREN), or options have been generated and we wish to
        continue trying to generate more (CHILDREN_AND_CONTINUE).

        If you want to cache something for this item for future calls, put it under plan['_'+item].

        Args:
            plan (dict): a partial plan
            item (str): an item to solve.

        Returns:
            tuple: a pair (status,children) where status is one of the StepResult codes
            FAIL, COMPLETE, CONTINUE, CHILDREN, CHILDREN_AND_CONTINUE, and
            children is a list of solutions that complete more of the plan.
        """
        return StepResult.FAIL, []

    def score(self, plan):
        """Returns a numeric score for the plan.  Default checks the score"""
        num_solved = len([k for k in plan if not k.startswith('_')])
        return plan.get('_solve_time', 0) - 0.2 * num_solved

    def assemble_result(self, plan):
        """Turns a dict partial plan into a complete result.  Default just
        returns the plan dict.
        """
        return dict((k, v) for (k, v) in plan.items() if not k.startwith('_'))

    def solve(self, tmax=float('inf')):
        """Solves the whole plan using least-commitment planning.

        Can be called multiple times to produce multiple solutions.
        """
        import heapq, copy
        if self.pending_solutions:
            soln = self.pending_solutions.pop(0)
            return self.assemble_result(soln)
        tstart = time.time()
        while len(self.W) > 0:
            if time.time() - tstart > tmax:
                return None
            node = heapq.heappop(self.W)
            plan = node.key
            prio = node.value
            item = self.choose_item(plan)
            if item is None:
                # no items left, done
                return self.assemble_result(plan)
            # print("Choosing item",item,"priority",prio)
            t0 = time.time()
            status, children = self.solve_item(plan, item)
            t1 = time.time()
            plan.setdefault('_solve_time', 0)
            plan['_solve_time'] += t1 - t0
            if status == StepResult.FAIL:
                continue
            elif status == StepResult.COMPLETE:
                assert len(children) > 0, "COMPLETE was returned but without an item solution?"
                soln = children[0]
                self.on_solve(plan, item, soln)
                plan[item] = soln
                for soln in children[1:]:
                    self.on_solve(plan, item, soln)
                    child = copy.copy(plan)
                    child[item] = soln
                    self.pending_solutions.append(child)
                return self.assemble_result(plan)
            else:
                if status == StepResult.CHILDREN_AND_CONTINUE or status == StepResult.CONTINUE:
                    heapq.heappush(self.W, PQNode(plan, self.score(plan)))
                for soln in children:
                    self.on_solve(plan, item, soln)
                    child = copy.copy(plan)
                    child[item] = soln
                    child['_solve_time'] = 0
                    heapq.heappush(self.W, PQNode(child, self.score(child)))
                    # print("Child priority",self.score(child))
        return None