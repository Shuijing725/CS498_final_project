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

        # set object to correct (R, t) in world frame
        vis.add('object transform:', object.getTransform())
        vis.add('Tobject_gripper', Tobject_gripper)
        # Tworld_gripper = robot1.link(gripper_link).getTransform()
        Tworld_gripper = gripper_link1.getTransform()
        Tworld_object = se3.mul(Tworld_gripper, Tobject_gripper)
        object.setTransform(Tworld_object[0], Tworld_object[1])
        vis.add('T world object', Tworld_object)

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

    Returns:
        None or (transfer,lower,ungrasp): giving the components of the place motion.
        Each element is a RobotTrajectory.  (Note: to convert a list of milestones
        to a RobotTrajectory, use RobotTrajectory(robot,milestones=milestones)

    Tip:
        vis.debug(q,world=world) will show a configuration.
    """
    def __init__(self,world,robot,object,Tobject_gripper,gripper,goal_bounds):
        MultiStepPlanner.__init__(self,['placement','qplace','qpreplace','retract','transfer'])
        self.world=world
        self.robot=robot
        self.object=object
        self.Tobject_gripper=Tobject_gripper
        self.gripper=gripper
        self.goal_bounds=goal_bounds

        object.setTransform(*se3.identity())
        self.objbb = object.geometry().getBBTight()
        self.qstart = robot.getConfig()  #lift

        # self.world.remove(self.world.terrain(7))

        self.world1 = self.world.copy()
        # remove the hammer object from the world
        self.world1.remove(self.world1.rigidObject(self.object.index))
        # find the robot in world1
        self.robot1 = self.world1.robot(0)
        # find the gripper link in new world????
        self.gripper_link1 = self.robot1.link(9)


    def object_free(self,q):
        """Helper: returns true if the object is collision free at configuration q, if it is
        attached to the gripper."""
        self.robot.setConfig(q)
        gripper_link = self.robot.link(self.gripper.base_link)
        self.object.setTransform(*se3.mul(gripper_link.getTransform(),self.Tobject_gripper))
        for i in range(self.world.numTerrains()):
            if self.object.geometry().collides(self.world.terrain(i).geometry()):
                return False
        for i in range(self.world.numRigidObjects()):
            if i == self.object.index: continue
            if self.object.geometry().collides(self.world.rigidObject(i).geometry()):
                return False
        return True

    def check_feasible(self):
        for i in range(self.world.numTerrains()):
            if self.object.geometry().collides(self.world.terrain(i).geometry()):
                return False

        for i in range(self.world.numRigidObjects()):
            if i == self.object.index: continue
            if self.object.geometry().collides(self.world.rigidObject(i).geometry()):
                return False
        return True

    def solve_placement(self):
        """Implemented for you: come up with a collision-free target placement"""
        # obmin,obmax = self.objbb
        # ozmin = obmin[2]
        # min_dims = min(obmax[0]-obmin[0],obmax[1]-obmin[1])
        # center = [0.5*(obmax[0]+obmin[0]),0.5*(obmax[1]-obmin[1])]
        xmin,ymin,zmin = self.goal_bounds[0]
        xmax,ymax,zmax = self.goal_bounds[1]
        #center_sampling_range = [(xmin+min_dims/2,xmax-min_dims/2),(ymin+min_dims/2,ymax-min_dims/2)]
        center_sampling_range = [(xmin, xmax), (ymin, ymax), (zmin, zmax)]
        Tobj_feasible = []
        original_obj_tarnsform=self.object.getTransform()
        for iters in range(20):
            crand = [random.uniform(b[0],b[1]) for b in center_sampling_range]
            #Robj = so3.rotation((0,0,1),random.uniform(0,math.pi*2))
            Robj=so3.identity()
            #tobj = vectorops.add(so3.apply(Robj,[-center[0],-center[1],0]),[crand[0],crand[1],zmin-ozmin+0.002])
            tobj=[crand[0], crand[1], crand[2]]


            self.object.setTransform(Robj,tobj)
            # vis.add("abdbdbd", tobj)

            if not self.check_feasible():
                continue
                # bmin,bmax = self.object.geometry().getBBTight()
                # if bmin[0] < xmin:
                #     tobj[0] += xmin-bmin[0]
                # if bmax[0] > xmax:
                #     tobj[0] -= bmin[0]-xmax
                # if bmin[1] < ymin:
                #     tobj[1] += ymin-bmin[1]
                # if bmax[1] > ymax:
                #     tobj[1] -= bmin[1]-ymax
                # self.object.setTransform(Robj,tobj)
                # feasible = True
                # for i in range(self.world.numTerrains()):
                #     if self.object.geometry().collides(self.world.terrain(i).geometry()):
                #         feasible=False
                #         break
                # if not feasible:
                #     continue

            Tobj_feasible.append((Robj,tobj))
        print("Found",len(Tobj_feasible),"valid object placements")
        self.object.setTransform(original_obj_tarnsform[0], original_obj_tarnsform[1])
        print("Tobj_feasible", Tobj_feasible)
        return Tobj_feasible



    def feasible(self):
        # collision check function
        finger_pad_links = ['gripper:Link_4', 'gripper:Link_6']

        finger_pad_links_idx = []
        for i in finger_pad_links:
            idx = self.robot1.link(i).getIndex()
            finger_pad_links_idx.append(idx)

        # set object to correct (R, t) in world frame

        # Tworld_gripper = robot1.link(gripper_link).getTransform()
        Tworld_gripper = self.gripper_link1.getTransform()
        Tworld_object = se3.mul(Tworld_gripper, self.Tobject_gripper)
        self.object.setTransform(Tworld_object[0], Tworld_object[1])


        # self collide
        if self.robot1.selfCollides():
            print('self collides')
            return False
        # collision between object and robot
        for i in range(self.robot1.numLinks()):
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

    def solve_qplace(self,Tplacement):
        vis.add('link 9', self.robot.link(9), color=[1, 0, 0, 1])
        print("TPla", Tplacement)
        # vis.add('goal', Tplacement[1], color=[1, 0, 0, 1])
        # Tplacement[1][0] = Tplacement[1][0] + 0.1
        vis.add('goal obj', Tplacement[1], color=[0, 1, 0, 1])
        print('Tobj_gripper',self.Tobject_gripper)
        # Tplacement1=se3.mul(Tplacement, se3.inv(self.Tobject_gripper))
        T_gripper_w=self.robot.link(9).getTransform()

        v=vectorops.add(T_gripper_w[1], [0,0,0.1+0.06])# se3.apply_rotation(T_gripper_w, [0,0,0.1+0.06])
        objective = ik.objective(self.robot.link(9), R=Tplacement[0], t=vectorops.add(Tplacement[1], v))

        # h = 0.1
        # ee_local_pos = (0, 0, 0)
        # # we want the end effector to grasp from top of the object
        # ee_local_axis = self.gripper.primary_axis  # (1, 0, 0)
        # # vis.add('origin', (0, 0, 0), color=(0, 1, 0, 1))
        # # vis.add('ee local axis', ee_local_axis, color=(0, 1, 0, 1))
        # point_local_array = np.array(ee_local_pos)
        # axis_local_array = np.array(ee_local_axis)
        # axis_world_array = [0, 0, 1]
        # # 0.5-0.008, 0.025-0.008, 0.8), (0.5+0.008, 0.025+0.008, 1.
        # goal_pos_world = [np.random.uniform(0.5-0.008, 0.5+0.008), np.random.uniform(0.025-0.008, 0.025+0.008),
        #                            np.random.uniform(0.8, 1)]
        #
        # axis_world_array = np.array(axis_world_array)
        # goal_pos_world = np.array(goal_pos_world)
        # vis.add('ik obj', goal_pos_world, color=(1, 0, 0, 1))
        # objective = ik.objective(self.robot.link(9),
        #                                    local=[ee_local_pos, list(point_local_array + h * axis_local_array)],
        #                                    world=[goal_pos_world, list(goal_pos_world + h * axis_world_array)])

        solved = ik.solve_global(objectives=objective, iters=50, numRestarts=5, activeDofs=[1, 2, 3, 4, 5, 6, 7])
        print(solved)
        if not solved:
            print('qplace')
            return None
        qpreplace = self.robot.getConfig()
        qplace = self.gripper.set_finger_config(qpreplace, self.gripper.partway_open_config(1))
        return qplace

    def solve_preplace(self,qplace):
        #TODO: solve for the preplacement configuration
        q_preplace=self.gripper.set_finger_config(qplace, self.gripper.partway_open_config(0.1))
        self.robot.setConfig(q_preplace)
        if not self.feasible():
            print('q_preplace')
            return None
        return q_preplace

    def solve_retract(self,qplace):
        #TODO: solve for the retraction step
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
        # self.qtransfer=retract(self.robot,self.gripper,amount=[0,0,0.3], local=False)
        self.qtransfer = qpreplace
        if self.qtransfer is None:
            print('qtransfer')
            return None

        self.robot.setConfig(self.qtransfer)
        if not self.feasible(): return None
        self.robot.setConfig(self.qstart)
        transfer = transfer_plan(self.world, self.robot, self.qtransfer, self.object, self.Tobject_gripper)

        return transfer

    def assemble_result(self,plan):
        transfer = plan['transfer']
        qplace = plan['qplace']
        qpreplace = plan['qpreplace']
        retract = plan['retract']

        # add a waypoint to place the swab vertically
        # secondary_axis_gripper = self.gripper.secondary_axis
        # if not isinstance(self.gripper, (int, str)):
        #     gripper1 = self.gripper.base_link
        # else:
        #     gripper1 = self.gripper
        # link = self.robot.link(gripper1)
        # R_gripper_w, _ = link.getTransform()
        # secondary_axis_world = so3.apply(R_gripper_w, secondary_axis_gripper)
        # secondary_axis_world_2d = np.array(secondary_axis_world)[:-1]
        # angle = np.arccos(np.dot(secondary_axis_world_2d, [0, 1]))
        angle = -np.pi/2
        q_rotate = copy.deepcopy(self.qtransfer)
        q_rotate[7] = q_rotate[7] - angle
        qpreplace[7] = qpreplace[7] - angle
        qplace[7] = qplace[7] - angle
        retract[7] = retract[7] - angle

        #TODO: construct the RobotTrajectory tuple (transfer,lower,retract)
        # todo: add a waypoint between qtransfer and qpreplace
        # return RobotTrajectory(self.robot, milestones=[self.qstart] + transfer), \
        #        RobotTrajectory(self.robot, milestones=[self.qtransfer, q_rotate, qpreplace, qplace]), \
        #        RobotTrajectory(self.robot, milestones=[qplace, retract])
        return RobotTrajectory(self.robot, milestones=[self.qstart] + transfer), \
               RobotTrajectory(self.robot, milestones=[self.qtransfer, q_rotate, qpreplace, qplace]), \
               RobotTrajectory(self.robot, milestones=[qplace])


    def solve_item(self,plan,item):
        if item == 'placement':
            Ts = self.solve_placement()
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


def plan_place(world,robot,obj,Tobject_gripper,gripper,goal_bounds):
    planner = PlacePlanner(world,robot,obj,Tobject_gripper,gripper,goal_bounds)
    # time_limit = 30
    return planner.solve()

