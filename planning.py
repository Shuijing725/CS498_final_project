from klampt.plan import robotplanning
from klampt.plan.cspace import MotionPlan
from klampt.model import trajectory
from klampt import vis 
from klampt import RobotModel
from klampt.plan import cspace

import math
import time
import numpy as np
import copy

def feasible_plan(world,robot,qtarget):
    """Plans for some number of iterations from the robot's current configuration to
    configuration qtarget.  Returns the first path found.

    Returns None if no path was found, otherwise returns the plan.
    """
    t0 = time.time()

    moving_joints = [1,2,3,4,5,6,7]


    kwargs = {'type':'sbl','perturbationRadius':2}
    planner = robotplanning.planToConfig(world=world, robot=robot, target=qtarget, edgeCheckResolution=0.01,
                                         movingSubset=moving_joints, **kwargs)
    # qtarget is not feasible
    if planner is None:
        return None
    path = None
    numIters = 0
    increment = 100  # there is a little overhead for each planMore call, so it is best not to set the increment too low
    t0 = time.time()
    while time.time() - t0 < 10:   #max 20 seconds of planning
        planner.planMore(increment)
        numIters = numIters + 1
        path = planner.getPath()
        if path is not None:
            if len(path) > 0:
                print("Solved, path has", len(path), "milestones")
                break

    t1 = time.time()
    print("Planning time,",numIters,"iterations",t1-t0)
    
    #to be nice to the C++ module, do this to free up memory
    planner.space.close()
    planner.close()

    return path


def optimizing_plan(world,robot,qtarget):
    """Plans for some number of iterations from the robot's current configuration to
    configuration qtarget.

    Returns None if no path was found, otherwise returns the best plan found.
    """
    t0 = time.time()

    moving_joints = [1, 2, 3, 4, 5, 6, 7]

    arg_list = [{'type':'sbl','shortcut':True,'perturbationRadius':0.5},
                {'type':'sbl','perturbationRadius':0.5,'shortcut':True,'restart':True,'restartTermCond':"{foundSolution:1,maxIters:100}"},
                {'type':'rrt*'},
                {'type':'lazyrrg*'}]
    planner_names = ['sbl1', 'sbl2', 'rrt*', 'lazyrrg']
    success = [0, 0, 0, 0]
    path_len = [[], [], [], []]

    best_path = None
    best_path_len = np.inf
    qinit = robot.getConfig()
    for i, kwarg in enumerate(arg_list):
        robot.setConfig(qinit)
        print('')
        print(planner_names[i])
        planner = robotplanning.planToConfig(world=world, robot=robot, target=qtarget, edgeCheckResolution=0.01,
                                             movingSubset=moving_joints, **kwarg)
        if planner is None:
            return None
        # there are 18 dimensions in config space, but we only allow the first 7 joints to move

        path = None
        numIters = 0
        increment = 100  # there is a little overhead for each planMore call, so it is best not to set the increment too low
        for j in range(10):
            t0 = time.time()
            while time.time() - t0 < 10:  # max 20 seconds of planning
                planner.planMore(increment)
                numIters = numIters + 1
                path = planner.getPath()
                if path is not None:
                    if len(path) > 0:
                        print("Solved, path has", len(path), "milestones")
                        success[i] = success[i] + 1
                        path_len[i].append(len(path))
                        # update best_path if a better path is found
                        if len(path) < best_path_len:
                            best_path = copy.deepcopy(path)
                            best_path_len = len(path)
                        break


        # to be nice to the C++ module, do this to free up memory
        planner.space.close()
        planner.close()

        print('success rate:', success[i]/10.)
        print('optimal path len:', min(path_len[i]), 'mean:', np.mean(path_len[i]), 'std:', np.std(path_len[i]))



    return best_path


def debug_plan_results(plan,robot):
    """Potentially useful for debugging planning results..."""
    assert isinstance(plan,MotionPlan)
    #this code just gives some debugging information. it may get expensive
    V,E = plan.getRoadmap()
    print(len(V),"feasible milestones sampled,",len(E),"edges connected")

    print("Plan stats:")
    pstats = plan.getStats()
    for k in sorted(pstats.keys()):
        print("  ",k,":",pstats[k])

    print("CSpace stats:")
    sstats = plan.space.getStats()
    for k in sorted(sstats.keys()):
        print("  ",k,":",sstats[k])
    """
    print("  Joint limit failures:")
    for i in range(robot.numLinks()):
        print("     ",robot.link(i).getName(),":",plan.space.ambientspace.joint_limit_failures[i])
    """

    path = plan.getPath()
    if path is None or len(path)==0:
        print("Failed to plan path between configuration")
        #debug some sampled configurations
        numconfigs = min(10,len(V))
        vis.debug("some milestones",V[2:numconfigs],world=world)
        pts = []
        for i,q in enumerate(V):
            robot.setConfig(q)
            pt = robot.link(9).getTransform()[1]
            pts.append(pt)
        for i,q in enumerate(V):
            vis.add("pt"+str(i),pts[i],hide_label=True,color=(1,1,0,0.75))
        for (a,b) in E:
            vis.add("edge_{}_{}".format(a,b),trajectory.Trajectory(milestones=[pts[a],pts[b]]),color=(1,0.5,0,0.5),width=1,pointSize=0,hide_label=True)
        return None

    print("Planned path with length",trajectory.RobotTrajectory(robot,milestones=path).length())
