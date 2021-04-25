from klampt import WorldModel
from klampt.io import resource
from klampt.model import ik
from klampt import vis 
from klampt.model.trajectory import RobotTrajectory
from klampt.math import vectorops,so3,se3
import pick
import math
import random
import sys
import numpy as np
sys.path.append("../common")
import grasp
import grasp_database
from known_grippers import *


if __name__ == '__main__':
    #load the robot / world file
    fn = "world.xml"
    world = WorldModel()
    res = world.readFile(fn)
    if not res:
        print("Unable to read file",fn)
        exit(0)

    #load the gripper info and grasp database
    source_gripper = robotiq_85
    target_gripper = robotiq_85_kinova_gen3

    db = grasp_database.GraspDatabase(source_gripper)
    if not db.load("grasps/robotiq_85_sampled_grasp_db.json"):
        raise RuntimeError("Can't load grasp database?")

    robot = world.robot(0)
    #need to fix the spin joints somewhat
    qmin,qmax = robot.getJointLimits()
    for i in range(len(qmin)):
        if qmax[i] - qmin[i] > math.pi*2:
            qmin[i] = -float('inf')
            qmax[i] = float('inf')
    robot.setJointLimits(qmin,qmax)

    obj = world.rigidObject('swab')
    #vis.add('', obj, color=[1, 0, 0, 1])

    #add the world elements individually to the visualization
    vis.add("world",world)
    qstart = resource.get("start.config",world=world)
    robot.setConfig(qstart)


    #transform all the grasps to use the kinova arm gripper and object transform
    orig_grasps = db.object_to_grasps["048_hammer"]
    sorted_grasps = sorted(orig_grasps, key=lambda grasp: grasp.score)
    grasp = sorted_grasps[2].get_transformed(obj.getTransform()).transfer(source_gripper,target_gripper)
    end_effector = robot.link('EndEffector_Link')
    h = 0.1
    ee_local_pos = (0, 0, 0)
    ee_local_axis = (1, 0, 0)
    point_local_array = np.array(ee_local_pos)
    axis_local_array = np.array(ee_local_axis)
    axis_world_array = np.array([0, 0, 1])
    goal_pos_world = (0.52, -0.175, 0.71+0.2)
    grasp.ik_constraint = ik.objective(end_effector, local=[ee_local_pos, list(point_local_array + h * axis_local_array)],
                                         world=[goal_pos_world, list(goal_pos_world + h * axis_world_array)])
    #this is just to do a nicer visualization... add the grasp and gripper to the visualizer
    # if PROBLEM == '2a':
    #     w2 = WorldModel()
    #     w2.loadFile(source_gripper.klampt_model)
    #     source_gripper_model = w2.robot(0)
    #     grasp_index = 2
    #     grasp = grasps[grasp_index]
    #     Tgripper = grasp.ik_constraint.closestMatch(*se3.identity())
    #     source_gripper_model.setConfig(orig_grasps[grasp_index].set_finger_config(source_gripper_model.getConfig()))
    #     source_gripper.add_to_vis(source_gripper_model,animate=False,base_xform=Tgripper)
    #     grasp.add_to_vis("grasp")
    # else:
    #     for i,grasp in enumerate(grasps):
    #         grasp.add_to_vis("grasp"+str(i))

    def planTriggered():
        global world,robot,obj,target_gripper,grasp
        qstart = robot.getConfig()
        res = pick.plan_pick_one(world,robot,obj,target_gripper,grasp)

        if res is None:
            print("Unable to plan pick")
        else:
            (transit,approach,lift) = res
            traj = transit
            traj = traj.concat(approach,relative=True,jumpPolicy='jump')
            traj = traj.concat(lift,relative=True,jumpPolicy='jump')
            vis.add("traj",traj,endEffectors=[9])
            vis.animate(vis.getItemName(robot),traj)
        robot.setConfig(qstart)

    vis.addAction(planTriggered,"Plan grasp",'p')

    # if PROBLEM == '2a':
    #     def shiftGrasp(amt):
    #         global grasp,grasps,grasp_index
    #         grasp_index += amt
    #         if grasp_index >= len(grasps):
    #             grasp_index = 0
    #         elif grasp_index < 0:
    #             grasp_index = len(grasps)-1
    #         print("Grasp",grasp_index)
    #         grasp = grasps[grasp_index]
    #         Tgripper = grasp.ik_constraint.closestMatch(*se3.identity())
    #         source_gripper_model.setConfig(orig_grasps[grasp_index].set_finger_config(source_gripper_model.getConfig()))
    #         source_gripper.add_to_vis(source_gripper_model,animate=False,base_xform=Tgripper)
    #         grasp.add_to_vis("grasp")
    #
    #     vis.addAction(lambda :shiftGrasp(1),"Next grasp",'g')
    vis.run()
