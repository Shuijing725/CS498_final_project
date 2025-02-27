from klampt.plan import robotplanning
from klampt.plan.cspace import MotionPlan
from klampt.model.trajectory import Trajectory,RobotTrajectory
from klampt.model import ik
from klampt.math import vectorops,so3,se3
from klampt import vis 
from klampt import RobotModel,Geometry3D
import sys
sys.path.append("../common")
from merge_geometry import merge_triangle_meshes
import math
import time
import random
from pick import is_collision_free_grasp,retract,MultiStepPlanner,StepResult
import numpy as np
import copy


def transfer_plan(world,robot,qtarget,object,Tobject_gripper):
    """Plans for some number of iterations from the robot's current configuration to
    configuration qtarget, assuming the object is fixed to the gripper. 
    Returns the first path found.

    Returns None if no path was found, otherwise returns the plan.
    """
    moving_joints = [1,2,3,4,5,6,7]
    gripper_link = 9

    # create a new copy of the world
    world1 = world.copy()
    # remove the hammer object from the world
    world1.remove(world1.rigidObject(object.index))
    # find the robot in world1
    robot1 = world1.robot(0)
    robot1.setConfig(robot.getConfig())
    # find the gripper link in new world????
    gripper_link1 = robot1.link(gripper_link)

    #kwargs = {'type':'rrt*', 'bidirectional':1}
    kwargs = {'type':'sbl','perturbationRadius':0.5, 'bidirectional': 1, 'shortcut':True,'restart':True,'restartTermCond':"{foundSolution:1,maxIters:100}"}
    t0 = time.time()

    # collision check function
    finger_pad_links = ['gripper:Link_4', 'gripper:Link_6']
    def check_collision(q):
        # set robot to current config
        robot1.setConfig(q)

        finger_pad_links_idx = []
        for i in finger_pad_links:
            idx = robot1.link(i).getIndex()
            finger_pad_links_idx.append(idx)

        Tworld_gripper = gripper_link1.getTransform()
        Tworld_object = se3.mul(Tworld_gripper, Tobject_gripper)
        object.setTransform(Tworld_object[0], Tworld_object[1])

        # self collide
        if robot1.selfCollides():
            print('self collides')
            return False
        # collision between object and robot
        for i in range(robot1.numLinks()):
            if i in finger_pad_links_idx:
                continue
            if robot1.link(i).geometry().collides(object.geometry()):
                print('hammer and robot', i)
                return False

        # collision between robot and world environment
        for i in range(world1.numTerrains()):
            for j in range(robot1.numLinks()):
                if robot1.link(j).geometry().collides(world1.terrain(i).geometry()):
                    print('robot and world')
                    return False
            if object.geometry().collides(world1.terrain(i).geometry()):
                print('hammer and world', i, world1.terrain(i).getName())
                return False

        # collision between robot and objects in world
        for i in range(world1.numRigidObjects()):
            for j in range(robot1.numLinks()):
                if world1.rigidObject(i) == object and j in finger_pad_links_idx:
                    continue
                if robot1.link(j).geometry().collides(world1.rigidObject(i).geometry()):
                    print('robot and other objects')
                    return False
            if object.geometry().collides(world1.rigidObject(i).geometry()):
                print('hammer and other objects')
                return False
        return True

    planner = robotplanning.planToConfig(world=world1, robot=robot1, target=qtarget, edgeCheckResolution=0.1,
                                         movingSubset=moving_joints, extraConstraints=[check_collision], **kwargs)

    if planner is None:
        return None

    # plan for 10 seconds
    path = None
    increment = 100  # there is a little overhead for each planMore call, so it is best not to set the increment too low
    t0 = time.time()
    while time.time() - t0 < 10:  # max 10 seconds of planning
        planner.planMore(increment)

        path = planner.getPath()
        if path is not None:
            if len(path) > 0:
                print("Solved, path has", len(path), "milestones")
                break

    t1 = time.time()
    print("Planning time,", t1 - t0)

    # to be nice to the C++ module, do this to free up memory
    planner.space.close()
    planner.close()
    print('path:', path)

    return path

class PlacePlanner(MultiStepPlanner):
    """
    Plans a placing motion for a given object and a specified grasp.

    Arguments:
        world (WorldModel): the world, containing robot, object, and other items that
            will need to be avoided.
        robot (RobotModel): the robot in its current configuration
        object (RigidObjectModel): the object to pick.
        Tobject_gripper (se3 object): transform of the object with respect to the gripper..
        goal_bounds (list): bounds of the goal region [(xmin,ymin,zmin,(xmax,ymax,zmax)]
        robot0: whether we are planning for the first robot (True) or the second robot (False)

    Returns:
        None or (transfer,lower,ungrasp): giving the components of the place motion.
        Each element is a RobotTrajectory.  (Note: to convert a list of milestones
        to a RobotTrajectory, use RobotTrajectory(robot,milestones=milestones)

    Tip:
        vis.debug(q,world=world) will show a configuration.
    """
    def __init__(self,world,robot,object,Tobject_gripper,gripper,goal_bounds, robot0=True):
        MultiStepPlanner.__init__(self,['placement','qplace','qpreplace','retract','transfer'])
        self.world=world
        self.robot=robot
        self.object=object
        self.Tobject_gripper=Tobject_gripper
        self.gripper=gripper
        self.goal_bounds=goal_bounds
        self.robot0 = robot0

        object.setTransform(*se3.identity())
        self.objbb = object.geometry().getBBTight()
        self.qstart = robot.getConfig()  #lift

        # self.world.remove(self.world.terrain(7))

        self.world1 = self.world.copy()
        # remove the hammer object from the world
        self.world1.remove(self.world1.rigidObject(self.object.index))
        # find the robot in world1
        self.robot1 = self.world1.robot(0)
        # find the gripper link in new world
        self.gripper_link1 = self.robot1.link(9)

        if self.robot0:
            self.close_amount = 0.1
        else:
            self.close_amount = 0.8

    def check_feasible(self):
        for i in range(self.world.numTerrains()):
            if self.object.geometry().collides(self.world.terrain(i).geometry()):
                return False

        for i in range(self.world.numRigidObjects()):
            print('check collision with', self.world.rigidObject(i).getName())
            if i == self.object.index: continue

            if self.object.geometry().collides(self.world.rigidObject(i).geometry()):
                print('collide!!!!')
                return False
        return True

    # Input: goal_bounds (list): bounds of the goal region [(xmin,ymin,zmin,(xmax,ymax,zmax)]
    # Output: a list of object transforms (R, t) that represent valid placement poses of the object
    def solve_placement(self, goal_bounds):
        """Implemented for you: come up with a collision-free target placement"""
        xmin,ymin,zmin = goal_bounds[0]
        xmax,ymax,zmax = goal_bounds[1]
        #center_sampling_range = [(xmin+min_dims/2,xmax-min_dims/2),(ymin+min_dims/2,ymax-min_dims/2)]
        center_sampling_range = [(xmin, xmax), (ymin, ymax), (zmin, zmax)]
        Tobj_feasible = []
        original_obj_tarnsform=self.object.getTransform()
        for iters in range(20):
            crand = [random.uniform(b[0],b[1]) for b in center_sampling_range]

            if self.robot0:
                Robj = so3.from_rpy((0, np.pi/2, 0))
            else:
                Robj = so3.identity()

            tobj=[crand[0], crand[1], crand[2]]

            self.object.setTransform(Robj,tobj)

            if not self.check_feasible():
                continue


            Tobj_feasible.append((Robj,tobj))
        print("Found",len(Tobj_feasible),"valid object placements")
        self.object.setTransform(original_obj_tarnsform[0], original_obj_tarnsform[1])
        print("Tobj_feasible", Tobj_feasible)
        return Tobj_feasible


    # callback function for checking whether the robot is collision-free, to be used in ik solver
    def feasible(self):
        finger_pad_links = ['gripper:Link_4', 'gripper:Link_6']

        finger_pad_links_idx = []
        for i in finger_pad_links:
            idx = self.robot1.link(i).getIndex()
            finger_pad_links_idx.append(idx)

        # set object to correct (R, t) in world frame
        Tworld_gripper = self.gripper_link1.getTransform()
        Tworld_object = se3.mul(Tworld_gripper, self.Tobject_gripper)
        self.object.setTransform(Tworld_object[0], Tworld_object[1])


        # self collide
        if self.robot1.selfCollides():
            print('self collides')
            return False
        # collision between object and robot
        for i in range(self.robot1.numLinks()):
            # ignore the collisions between finger links and the object to grasp
            if i in finger_pad_links_idx:
                continue
            if self.robot1.link(i).geometry().collides(self.object.geometry()):
                print('hammer and robot', i)
                return False

        # collision between robot and world environment
        for i in range(self.world1.numTerrains()):
            for j in range(self.robot1.numLinks()):
                if self.robot1.link(j).geometry().collides(self.world1.terrain(i).geometry()):
                    print('robot and world')
                    return False
            if self.object.geometry().collides(self.world1.terrain(i).geometry()):
                print('hammer and world')
                return False

        # collision between robot and objects in world
        for i in range(self.world1.numRigidObjects()):
            for j in range(self.robot1.numLinks()):
                if self.world1.rigidObject(i) == self.object and j in finger_pad_links_idx:
                    continue
                if self.robot1.link(j).geometry().collides(self.world1.rigidObject(i).geometry()):
                    print('robot and other objects')
                    return False
            if self.object.geometry().collides(self.world1.rigidObject(i).geometry()):
                print('hammer and other objects')
                return False
        return True

    # given a desired placement pose of the target object (Tplacement), find a placement configuration for the robot
    def solve_qplace(self,Tplacement):
        if self.robot0:
            v = list(np.array(Tplacement[1]) - np.array([0.16, 0, 0]))
            objective = ik.objective(self.robot.link(9), R=Tplacement[0], t=v)
            solved = ik.solve_global(objectives=objective, iters=50, numRestarts=5, activeDofs=[1, 2, 3, 4, 5, 6, 7],
                                     feasibilityCheck=self.feasible)

            if not solved:
                print('qplace')
                return None
            qpreplace = self.robot.getConfig()

            # add a waypoint to insert swab into human face
            face_trans = [0.34, -0.3, 1.05]# [0.5, -0.35, 1.35]
            objective = ik.objective(self.robot.link(9), R=so3.from_rpy((np.pi, -np.pi/2, 0)), t=face_trans)
            solved = ik.solve_global(objectives=objective, iters=50, numRestarts=5, activeDofs=[1, 2, 3, 4, 5, 6, 7],
                                     feasibilityCheck=self.feasible)
            if not solved:
                print('cannot place swab near mouth')
                return None
            self.qmouth = self.robot.getConfig()
            self.qmouth = self.gripper.set_finger_config(self.qmouth,
                                                         self.gripper.partway_open_config(self.close_amount))

            face_trans = [0.34, -0.4, 1.05]  # [0.5, -0.35, 1.35]
            objective = ik.objective(self.robot.link(9), R=so3.from_rpy((np.pi, -np.pi/2, 0)), t=face_trans)
            solved = ik.solve_global(objectives=objective, iters=50, numRestarts=5, activeDofs=[1, 2, 3, 4, 5, 6, 7])
            if not solved:
                print('cannot insert swab into mouth')
                return None
            self.qmouth_in = self.robot.getConfig()
            self.qmouth_in = self.gripper.set_finger_config(self.qmouth_in,
                                                         self.gripper.partway_open_config(self.close_amount))

            # add a waypoint to lower down the gripper
            v1 = list(np.array(v)-np.array([0, 0, 0.1]))
            # vis.add('v1', v1, color=[0, 0, 1, 1])
            objective = ik.objective(self.robot.link(9), R=so3.from_rpy((-np.pi/2, 0, -np.pi/2)), t=v1)
            solved = ik.solve_global(objectives=objective, iters=50, numRestarts=5, activeDofs=[1, 2, 3, 4, 5, 6, 7],
                                     feasibilityCheck=self.feasible)
            if not solved:
                print('qlower')
                return None
            self.qlower = self.robot.getConfig()
            self.qlower = self.gripper.set_finger_config(self.qlower, self.gripper.partway_open_config(self.close_amount))


            # add waypoints to dip the swab into the plate
            goal = (0.7, 0.35, 0.9)
            t = list(np.array(goal)-np.array([0.16, 0, 0]))
            objective = ik.objective(self.robot.link(9), R=so3.from_rpy((-np.pi/2, np.pi/2, -np.pi/2)), t=t)
            solved = ik.solve_global(objectives=objective, iters=50, numRestarts=5, activeDofs=[1, 2, 3, 4, 5, 6, 7],
                                     feasibilityCheck=self.feasible)
            if not solved:
                print('cannot place swab into plate')
                return None
            self.qplaceplate = self.robot.getConfig()
            self.qplaceplate = self.gripper.set_finger_config(self.qplaceplate,
                                                         self.gripper.partway_open_config(self.close_amount))


            t1 = list(np.array(t)-np.array([0, 0, 0.06]))
            objective = ik.objective(self.robot.link(9), R=so3.from_rpy((-np.pi / 2, np.pi / 2, -np.pi / 2)),
                                     t=t1)
            solved = ik.solve_global(objectives=objective, iters=50, numRestarts=5, activeDofs=[1, 2, 3, 4, 5, 6, 7],
                                     feasibilityCheck=self.feasible)
            if not solved:
                print('cannot place swab into plate')
                return None
            self.qplaceplate_lower = self.robot.getConfig()
            self.qplaceplate_lower = self.gripper.set_finger_config(self.qplaceplate_lower,
                                                              self.gripper.partway_open_config(self.close_amount))

            # add waypoints to throw the swab into the trash can
            goal = (0.1, -0.4, 0.8)
            t = list(np.array(goal)-np.array([0.16, 0, 0]))
            objective = ik.objective(self.robot.link(9), R=so3.from_rpy((-np.pi/2, np.pi/2, -np.pi/2)), t=t)
            solved = ik.solve_global(objectives=objective, iters=50, numRestarts=5, activeDofs=[1, 2, 3, 4, 5, 6, 7],
                                     feasibilityCheck=self.feasible)
            if not solved:
                print('cannot place swab into plate')
                return None
            self.qplace_trash = self.robot.getConfig()
            self.qplace_trash = self.gripper.set_finger_config(self.qplace_trash, self.gripper.partway_open_config(self.close_amount))
            self.qplace = self.gripper.set_finger_config(self.qplace_trash, self.gripper.partway_open_config(1))

        else:
            Tplacement_x = se3.mul(Tplacement, se3.inv(self.Tobject_gripper))
            objective = ik.objective(self.robot.link(9), R=Tplacement_x[0], t=Tplacement_x[1])
            solved = ik.solve_global(objectives=objective,
                                     iters=50,
                                     numRestarts=5,
                                     activeDofs=[1, 2, 3, 4, 5, 6, 7],
                                     feasibilityCheck=self.feasible)
            if not solved: return None
            qpreplace = self.robot.getConfig()

        qplace = self.gripper.set_finger_config(qpreplace, self.gripper.partway_open_config(1))

        return qplace

    def solve_preplace(self,qplace):
        #TODO: solve for the preplacement configuration
        q_preplace=self.gripper.set_finger_config(qplace, self.gripper.partway_open_config(self.close_amount))
        self.robot.setConfig(q_preplace)
        if not self.feasible():
            print('q_preplace')
            return None
        return q_preplace

    def solve_retract(self,qplace):
        # TODO: solve for the retraction step
        self.robot.setConfig(qplace)

        qretract=retract(self.robot, self.gripper, amount=list(-0.1 * np.array(self.gripper.primary_axis)), local=True)
        if qretract is None:
            print('qretract')
            return None
        self.robot.setConfig(qretract)
        if not self.feasible(): return None

        return qretract

    def solve_transfer(self,qpreplace):
        #TODO: solve for the transfer plan
        self.robot.setConfig(qpreplace)
        self.qtransfer = copy.deepcopy(qpreplace)
        if self.qtransfer is None:
            print('qtransfer')
            return None

        self.robot.setConfig(self.qtransfer)
        if not self.feasible(): return None
        if self.robot0:
            self.robot.setConfig(self.qmouth)
        else:
            self.robot.setConfig(self.qstart)
        transfer = transfer_plan(self.world, self.robot, self.qtransfer, self.object, self.Tobject_gripper)

        return transfer

    def assemble_result(self,plan):
        transfer = plan['transfer']
        qplace = plan['qplace']
        qpreplace = plan['qpreplace']
        retract = plan['retract']

        #TODO: construct the RobotTrajectory tuple (transfer,lower,retract)
        if self.robot0:
            angle = -np.pi / 2
            q_rotate = copy.deepcopy(self.qtransfer)
            q_rotate[7] = q_rotate[7] - angle
            qpreplace[7] = qpreplace[7] - angle
            self.qplaceplate[7] = self.qplaceplate[7] - angle
            self.qplaceplate_lower[7] = self.qplaceplate_lower[7] - angle
            self.qplace_trash[7] = self.qplace_trash[7] - angle
            self.qplace[7] = self.qplace[7] - angle

            # plan a path between qpreplace and qlower
            self.robot.setConfig(qpreplace)
            self.lower_plan = transfer_plan(self.world, self.robot, self.qlower, self.object, self.Tobject_gripper)

            return RobotTrajectory(self.robot, milestones=[self.qstart, self.qmouth, self.qmouth_in, self.qmouth] + transfer), \
                   RobotTrajectory(self.robot,
                                   milestones=[self.qtransfer, q_rotate] + self.lower_plan + self.lower_plan[::-1]+
                                               [self.qplaceplate,self.qplaceplate_lower, self.qplaceplate, self.qplace_trash,
                                               self.qplace]), \
                   RobotTrajectory(self.robot, milestones=[self.qplace])

        else:
            return RobotTrajectory(self.robot, milestones=[self.qstart] + transfer), \
                   RobotTrajectory(self.robot, milestones=[self.qtransfer, qpreplace, qplace]), \
                   RobotTrajectory(self.robot, milestones=[qplace, retract])




    def solve_item(self,plan,item):
        if item == 'placement':
            Ts = self.solve_placement(self.goal_bounds)
            return StepResult.CHILDREN_AND_CONTINUE,Ts
        if item == 'qplace':
            qplace = self.solve_qplace(plan['placement'])
            if qplace is None:
                return StepResult.CONTINUE,[]
            else:
                return StepResult.CHILDREN_AND_CONTINUE,[qplace]
        if item == 'qpreplace':
            qpreplace = self.solve_preplace(plan['qplace'])
            if qpreplace is None:
                return StepResult.FAIL,[]
            else:
                return StepResult.CHILDREN,[qpreplace]
        if item == 'retract':
            retract = self.solve_retract(plan['qplace'])
            if retract is None:
                return StepResult.FAIL,[]
            else:
                return StepResult.CHILDREN,[retract]
        if item == 'transfer':
            transfer = self.solve_transfer(plan['qpreplace'])
            if transfer is None:
                return StepResult.CONTINUE,[]
            else:
                return StepResult.CHILDREN,[transfer]
        raise ValueError("Invalid item "+item)


def plan_place(world,robot,obj,Tobject_gripper,gripper,goal_bounds, robot0):
    planner = PlacePlanner(world,robot,obj,Tobject_gripper,gripper,goal_bounds, robot0=robot0)
    # time_limit = 30
    return planner.solve()

