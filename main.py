from klampt import WorldModel,Simulator
from klampt.io import resource
from klampt.model import ik,sensing
from klampt import vis
from klampt.model.trajectory import Trajectory,RobotTrajectory,path_to_trajectory
from klampt.math import vectorops,so3,se3
import pick
import place
import math
import random
import sys
import numpy as np
import time
import copy
import json
sys.path.append("../common")
import grasp
import grasp_database
from known_grippers import *

# physics simulation doesn't work for now, set it to False!
PHYSICS_SIMULATION = False

if __name__ == '__main__':
    #load the robot / world file
    fn = "world.xml"
    world = WorldModel()
    res = world.readFile(fn)
    if not res:
        print("Unable to read file",fn)
        exit(0)

    for i in range(world.numRigidObjects()):
        obj = world.rigidObject(i)
        # this will perform a reasonable center of mass / inertia estimate
        m = obj.getMass()
        m.estimate(obj.geometry(),mass=0.454,surfaceFraction=0.2)
        obj.setMass(m)

    plate_mass = world.rigidObject("plate").getMass()
    plate_mass.setInertia([1,0,0,0,1,0,0,0,1])
    world.rigidObject("plate").setMass(plate_mass)
    plate_mass = world.rigidObject("pcr").getMass()
    plate_mass.setInertia([1,0,0,0,1,0,0,0,1])
    world.rigidObject("pcr").setMass(plate_mass)
    
    # load the gripper info and grasp database
    source_gripper = robotiq_85
    target_gripper = robotiq_85_kinova_gen3

    db = grasp_database.GraspDatabase(source_gripper)
    if not db.load("grasps/robotiq_85_sampled_grasp_db.json"):
        raise RuntimeError("Can't load grasp database?")

    robot = world.robot(0)
    robot2 = world.robot(1)
    gripper_link = robot.link(9)
    #need to fix the spin joints somewhat
    qmin,qmax = robot.getJointLimits()
    for i in range(len(qmin)):
        if qmax[i] - qmin[i] > math.pi*2:
            qmin[i] = -float('inf')
            qmax[i] = float('inf')
    robot.setJointLimits(qmin,qmax)
    qmin, qmax = robot2.getJointLimits()
    for i in range(len(qmin)):
        if qmax[i] - qmin[i] > math.pi * 2:
            qmin[i] = -float('inf')
            qmax[i] = float('inf')
    robot2.setJointLimits(qmin, qmax)
    swab = world.rigidObject('swab')
    plate = world.rigidObject('plate')

    #add the world elements individually to the visualization
    vis.add("world",world)
    qstart = resource.get("start.config",world=world)
    robot.setConfig(qstart)
    robot2.setConfig(qstart)



    #transform all the grasps to use the kinova arm gripper and object transform
    orig_grasps = db.object_to_grasps["048_hammer"]
    sorted_grasps = sorted(orig_grasps, key=lambda grasp: grasp.score)
    grasp_swab = sorted_grasps[2].get_transformed(swab.getTransform()).transfer(source_gripper,target_gripper)
    grasp_plate = grasp.Grasp(ik_constraint=sorted_grasps[2].ik_constraint,finger_links=sorted_grasps[2].finger_links,
                              finger_config=sorted_grasps[2].finger_config,contacts=sorted_grasps[2].contacts)
    grasp_plate = grasp_plate.get_transformed(plate.getTransform()).transfer(source_gripper,target_gripper)

    end_effector = robot.link('EndEffector_Link')
    h = 0.1
    ee_local_pos = (0, 0, 0)
    # we want the end effector to grasp from top of the object
    ee_local_axis = source_gripper.primary_axis # (1, 0, 0)
    point_local_array = np.array(ee_local_pos)
    axis_local_array = np.array(ee_local_axis)
    axis_world_array = np.array([0, 0, -1])
    # grasp for swab
    goal_pos_world = (0.5, -0.175, 0.71+0.17)
    grasp_swab.ik_constraint = ik.objective(end_effector, local=[ee_local_pos, list(point_local_array + h * axis_local_array)],
                                         world=[goal_pos_world, list(goal_pos_world + h * axis_world_array)])
    # grasp for plate
    goal_pos_world_plate = (0.68, 0.35, 0.87)
    vis.add('', goal_pos_world_plate, color=[1, 0, 0, 1])
    end_effector2 = robot2.link('EndEffector_Link')
    grasp_plate.ik_constraint = ik.objective(end_effector2, local=[ee_local_pos, list(point_local_array + h * axis_local_array)],
                                         world=[goal_pos_world_plate, list(goal_pos_world_plate + h * axis_world_array)])

    solved_trajectory = None
    trajectory_is_transfer = None
    next_robot_to_move = 0
    T_gripper_w = robot.link(9).getTransform()

    # Plan paths for the two robots
    def planTriggered():
        global world,robot, robot2, swab,target_gripper,grasp_swab, grasp_plate, solved_trajectory, \
            trajectory_is_transfer, next_robot_to_move
        ########################## Plan for robot0 & swab ############################
        if next_robot_to_move == 0:
            # plan pick
            Tobj0 = swab.getTransform()
            # center of tube: (0.5 0.025 0.72)
            offset = 0.005
            goal_bounds = [(0.615 - offset, 0.025 - offset/4, 0.85), (0.615 + offset, 0.025 +offset/4, 1.0)]
            qstart = robot.getConfig()
            res = pick.plan_pick_one(world, robot, swab, target_gripper, grasp_swab, robot0=True)

            if res is None:
                print("Unable to plan pick")
            else:
                print("plan pick success")
                transit, approach, lift = res
                # plan place
                qgrasp = lift.milestones[-1]
                robot.setConfig(qgrasp)
                Tobj = swab.getTransform()  # swab in world frame
                # swab in gripper frame
                Tobject_grasp = se3.mul(se3.inv(gripper_link.getTransform()), Tobj)
                place.robot0 = True
                res = place.plan_place(world, robot, swab, Tobject_grasp, target_gripper, goal_bounds, True)
                robot.setConfig(qstart)
                if res is None:
                    print("Unable to plan place")
                else:
                    (transfer, lower, retract) = res
                    trajectory_is_transfer = Trajectory()
                    trajectory_is_transfer.times.append(0)
                    trajectory_is_transfer.milestones.append([0])
                    traj = transit
                    traj = traj.concat(approach, relative=True, jumpPolicy='jump')
                    trajectory_is_transfer.times.append(traj.endTime())
                    trajectory_is_transfer.times.append(traj.endTime())
                    trajectory_is_transfer.milestones.append([0])
                    trajectory_is_transfer.milestones.append([1])
                    traj = traj.concat(lift, relative=True, jumpPolicy='jump')
                    traj = traj.concat(transfer, relative=True, jumpPolicy='jump')
                    traj = traj.concat(lower, relative=True, jumpPolicy='jump')
                    trajectory_is_transfer.times.append(traj.endTime())
                    trajectory_is_transfer.times.append(traj.endTime())
                    trajectory_is_transfer.milestones.append([1])
                    trajectory_is_transfer.milestones.append([0])

                    solved_trajectory = traj

                    swab.setTransform(*Tobj0)

                    vis.add("traj", traj, endEffectors=[gripper_link.index])

        ############################# Plan for Plate & robot 2 starts here! #################################
        elif next_robot_to_move == 1:
            Tobj0 = plate.getTransform()
            goal_bounds = [(0.7, 0.62, 0.71), (0.7, 0.62, 0.71)]
            res = pick.plan_pick_one(world, robot2, plate, target_gripper, grasp_plate, robot0=False)
            if res is None:
                print("Unable to plan pick")
            else:
                print("plan pick success")
                transit, approach, lift = res

                qgrasp = lift.milestones[-1]
                robot2.setConfig(qgrasp)
                Tobj = plate.getTransform()  # plate in world frame
                # plate in gripper frame
                Tobject_grasp = se3.mul(se3.inv(robot2.link(9).getTransform()), Tobj)
                place.robot0 = False
                res = place.plan_place(world, robot2, plate, Tobject_grasp, target_gripper, goal_bounds, False)
                if res is None:
                    print("Unable to plan place")
                else:
                    (transfer, lower, retract) = res
                    trajectory_is_transfer = Trajectory()
                    trajectory_is_transfer.times.append(0)
                    trajectory_is_transfer.milestones.append([0])
                    traj = transit
                    traj = traj.concat(approach, relative=True, jumpPolicy='jump')
                    trajectory_is_transfer.times.append(traj.endTime())
                    trajectory_is_transfer.times.append(traj.endTime())
                    trajectory_is_transfer.milestones.append([0])
                    trajectory_is_transfer.milestones.append([1])
                    traj = traj.concat(lift, relative=True, jumpPolicy='jump')
                    traj = traj.concat(transfer, relative=True, jumpPolicy='jump')
                    traj = traj.concat(lower, relative=True, jumpPolicy='jump')
                    trajectory_is_transfer.times.append(traj.endTime())
                    trajectory_is_transfer.times.append(traj.endTime())
                    trajectory_is_transfer.milestones.append([1])
                    trajectory_is_transfer.milestones.append([0])
                    traj = traj.concat(retract, relative=True, jumpPolicy='jump')
                    trajectory_is_transfer.times.append(traj.endTime())
                    trajectory_is_transfer.milestones.append([0])
                    solved_trajectory = traj

                    plate.setTransform(*Tobj0)

                    vis.add("traj", traj, endEffectors=[robot2.link(9).index])



    vis.addAction(planTriggered,"Plan grasp",'p')


    executing_plan = False
    execute_start_time = None

    # execute the planned path
    def executePlan():
        global solved_trajectory, trajectory_is_transfer, executing_plan, execute_start_time
        if solved_trajectory is None:
            return
        executing_plan = True
        if PHYSICS_SIMULATION:
            execute_start_time = 0
            solved_trajectory = path_to_trajectory(solved_trajectory, timing='robot', smoothing=None)
            solved_trajectory.times = [10 * t for t in solved_trajectory.times]
        else:
            execute_start_time = time.time()


    vis.addAction(executePlan, "Execute plan", 'e')


    sim = Simulator(world)
    sim_dt = 0.02
    sensor = sim.controller(0).sensor("realsense")
    sensor2 = sim.controller(0).sensor("realsense")

    # vis.add("sensor", sensor)
    was_grasping = False


    swab_time = 0
    swab_init_height = 0
    swab_height = 0

    def loop_callback():
        global was_grasping, Tobject_grasp, solved_trajectory, trajectory_is_transfer, executing_plan, \
            execute_start_time, qstart, next_robot_to_move, swab_time, swab_init_height, swab_height

        global sensor
        sensor.kinematicReset()
        sensor.kinematicSimulate(world,0.01)
        if next_robot_to_move == 0:
            cur_robot = robot
            cur_obj = swab
        elif next_robot_to_move == 1:
            cur_robot = robot2
            cur_obj = plate

        if not executing_plan:
            return
        if PHYSICS_SIMULATION:
            execute_start_time += sim_dt
            t = execute_start_time
        else:
            t = time.time() - execute_start_time
        vis.addText("time", "Time %.3f" % (t), position=(10, 10))

        qstart = solved_trajectory.eval(t)
        if PHYSICS_SIMULATION:
            # sim.controller(0).setPIDCommand(qstart,solved_trajectory.deriv(t))
            # sim.controller(0).setMilestone(qstart)
            sim.controller(0).setLinear(qstart, sim_dt * 5)
            sim.simulate(sim_dt)
            sim.updateWorld()
        else:
            cur_robot.setConfig(qstart)
            during_transfer = trajectory_is_transfer.eval(t)[0]
            if not was_grasping:
                # pick up object
                Tobject_grasp = se3.mul(se3.inv(cur_robot.link(9).getTransform()), cur_obj.getTransform())
                was_grasping = True
            if during_transfer:
                cur_obj.setTransform(*se3.mul(cur_robot.link(9).getTransform(), Tobject_grasp))
            else:
                was_grasping = False
        if t > solved_trajectory.duration()-0.8:
            swab_init_height = swab.getTransform()[1][2]
            swab_height = copy.deepcopy(swab_init_height)
        if t > solved_trajectory.duration() + 1:
            executing_plan = False
            solved_trajectory = None
            vis.add('gripper end', cur_robot.link(9).getTransform())
            next_robot_to_move += 1
        # let swab drop into trash can
        if solved_trajectory is not None and next_robot_to_move == 0:
            if solved_trajectory.duration()-0.8 < t < solved_trajectory.duration() + 1:
                if swab_height > 0:
                    swab_height = swab_init_height - 0.5 * 1 * swab_time**2
                    swab_time = swab_time + sim_dt
                Rs, ts = swab.getTransform()
                ts[2] = swab_height
                swab.setTransform(Rs, ts)

    vis.loop(callback=loop_callback)
