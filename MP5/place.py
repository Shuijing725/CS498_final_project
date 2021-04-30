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
from planning import *

def extraConstraints(q, world,robot,qtarget,object,Tobject_gripper):
    robot.setConfig(q)
    if robot.selfCollides():
        return False
    T_o_w = se3.mul(robot.link(9).getTransform(), Tobject_gripper)
    object.setTransform(*T_o_w)
    for i in range(world.numTerrains()):
        if object.geometry().collides(world.terrain(i).geometry()):
            return False
    for i in range(world.numRigidObjects()):
        if object.geometry().collides(world.rigidObject(i).geometry()):
            return False
    for i in range(world.numTerrains()):
        for j in range(robot.numLinks()):
            if robot.link(j).getName() == "gripper:Link_4" or robot.link(j).getName() == "gripper:Link_6":
                continue
            if robot.link(j).geometry().collides(world.terrain(i).geometry()):
                return False
    for i in range(world.numRigidObjects()):
        for j in range(robot.numLinks()):
            if robot.link(j).getName() == "gripper:Link_4" or robot.link(j).getName() == "gripper:Link_6":
                continue
            if robot.link(j).geometry().collides(world.rigidObject(i).geometry()):
                return False
    return True


def transfer_plan(world,robot,qtarget,object,Tobject_gripper):
    """Plans for some number of iterations from the robot's current configuration to
    configuration qtarget, assuming the object is fixed to the gripper. 
    Returns the first path found.

    Returns None if no path was found, otherwise returns the plan.
    """
    moving_joints = [1,2,3,4,5,6,7]
    gripper_link = 9

    #TODO: how do you implement this?
    qstart = robot.getConfig()
    
    remove_object_world = world.copy()
    remove_object_world.remove(remove_object_world.rigidObject(object.index))
    
    planner = robotplanning.planToConfig(remove_object_world,remove_object_world.robot(0),qtarget,type='sbl',perturbationRadius=0.5,bidirectional=True, shortcut=True,restart=True,restartTermCond="{foundSolution:1,maxIters:100}",extraConstraints=[lambda q:extraConstraints(q, remove_object_world, remove_object_world.robot(0), qtarget,object,Tobject_gripper)])
    
    increment = 100                #there is a little overhead for each planMore call, so it is best not to set the increment too low
    t0 = time.time()
    while time.time() - t0 < 10:   #max 20 seconds of planning
        planner.planMore(increment)
        path = planner.getPath()
        if path and len(path) > 0:
            print("Solved, path has",len(path),"milestones")
            print("Took time",time.time()-t0)
            break
    planner.close()   #frees a little memory... this is only really necessary if you are creating lots of planners
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

    def solve_placement(self):
        """Implemented for you: come up with a collision-free target placement"""
        obmin,obmax = self.objbb
        ozmin = obmin[2]
        min_dims = min(obmax[0]-obmin[0],obmax[1]-obmin[1])
        center = [0.5*(obmax[0]+obmin[0]),0.5*(obmax[1]-obmin[1])]
        xmin,ymin,zmin = self.goal_bounds[0]
        xmax,ymax,zmax = self.goal_bounds[1]
        center_sampling_range = [(xmin+min_dims/2,xmax-min_dims/2),(ymin+min_dims/2,ymax-min_dims/2)]
        Tobj_feasible = []
        for iters in range(20):
            crand = [random.uniform(b[0],b[1]) for b in center_sampling_range]
            Robj = so3.rotation((0,0,1),random.uniform(0,math.pi*2))
            tobj = vectorops.add(so3.apply(Robj,[-center[0],-center[1],0]),[crand[0],crand[1],zmin-ozmin+0.002])
            self.object.setTransform(Robj,tobj)
            feasible = True
            for i in range(self.world.numTerrains()):
                if self.object.geometry().collides(self.world.terrain(i).geometry()):
                    feasible=False
                    break
            if not feasible:
                bmin,bmax = self.object.geometry().getBBTight()
                if bmin[0] < xmin:
                    tobj[0] += xmin-bmin[0]
                if bmax[0] > xmax:
                    tobj[0] -= bmin[0]-xmax
                if bmin[1] < ymin:
                    tobj[1] += ymin-bmin[1]
                if bmax[1] > ymax:
                    tobj[1] -= bmin[1]-ymax
                self.object.setTransform(Robj,tobj)
                feasible = True
                for i in range(self.world.numTerrains()):
                    if self.object.geometry().collides(self.world.terrain(i).geometry()):
                        feasible=False
                        break
                if not feasible:
                    continue
            for i in range(self.world.numRigidObjects()):
                if i == self.object.index: continue
                if self.object.geometry().collides(self.world.rigidObject(i).geometry()):
                    #raise it up a bit
                    bmin,bmax = self.world.rigidObject(i).geometry().getBB()
                    tobj[2] = bmax[2]-ozmin+0.002
                    self.object.setTransform(Robj,tobj)
            Tobj_feasible.append((Robj,tobj))
        print("Found",len(Tobj_feasible),"valid object placements")
        return Tobj_feasible
    
    def solve_qplace(self,Tplacement):
        #TODO: solve for the placement configuration
        T_g_w = se3.mul(Tplacement, se3.inv(self.Tobject_gripper))
        obje = ik.objective(self.robot.link(self.gripper.base_link), R=T_g_w[0], t=T_g_w[1])
        solved = ik.solve_global(obje, feasibilityCheck=lambda: is_collision_free_grasp(self.world, self.robot, self.object))
        if solved:
            qplace = self.robot.getConfig()
            return qplace
        pass

    def solve_preplace(self,qplace):
        #TODO: solve for the preplacement configuration
        self.robot.setConfig(qplace)
        gripper_link = self.robot.link(self.gripper.base_link)
        p = gripper_link.getWorldPosition((0,0,0))
        d = gripper_link.getWorldDirection((0,0,1))
        obje = ik.objective(self.robot.link(self.gripper.base_link), local=[(0,0,0),(0,0,1)], world=[vectorops.add(p,(0,0,0.1)), vectorops.add(vectorops.add(p,(0,0,0.1)),d)])
        solved = ik.solve_global(obje, feasibilityCheck=lambda: is_collision_free_grasp(self.world, self.robot, self.object))
    
        qlift = self.robot.getConfig()
        return qlift
        pass

    def solve_retract(self,qplace):
        #TODO: solve for the retraction step
        self.robot.setConfig(qplace)
        gripper_link = self.robot.link(self.gripper.base_link)
        p = gripper_link.getWorldPosition((0,0,0))
        d = gripper_link.getWorldDirection((0,0,1))
        obje = ik.objective(self.robot.link(self.gripper.base_link), local=[(0,0,0),(0,0,1)], world=[vectorops.madd(p,d,-0.1), vectorops.madd(p,d,0.9)])
        solved = ik.solve_global(obje, feasibilityCheck=lambda: is_collision_free_grasp(self.world, self.robot, self.object))
    
        qpregrasp = self.robot.getConfig()   #TODO solve the retraction problem for qpregrasp?
        return qpregrasp

    def solve_transfer(self,qpreplace):
        #TODO: solve for the transfer plan
        self.robot.setConfig(self.qstart)
        transit = feasible_plan(self.world,self.robot,qpreplace)
        
        
        return transit
        pass

    def assemble_result(self,plan):
        transfer = plan['transfer']
        qplace = plan['qplace']
        qpreplace = plan['qpreplace']
        retract = plan['retract']
        #TODO: construct the RobotTrajectory tuple (transfer,lower,retract)
        return (
            RobotTrajectory(self.robot,milestones=transfer+[qpreplace])
            ,RobotTrajectory(self.robot,milestones=[qpreplace] + [qplace])
            ,RobotTrajectory(self.robot,milestones=[qplace,retract])
        )
        return plan

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
    time_limit = 60
    return planner.solve(time_limit)
