from klampt.plan import robotplanning, motionplanning
from klampt.plan.cspace import MotionPlan
from klampt.model import trajectory
from klampt import vis 
from klampt import RobotModel
import math
import time
import numpy as np

def feasible_plan(world,robot,qtarget):
    """Plans for some number of iterations from the robot's current configuration to
    configuration qtarget.  Returns the first path found.

    Returns None if no path was found, otherwise returns the plan.
    """
    t0 = time.time()

    moving_joints = [1,2,3,4,5,6,7]
    space = robotplanning.makeSpace(world=world,robot=robot,edgeCheckResolution=1e-2,movingSubset=moving_joints)
    #plan = MotionPlan(space,type='sbl',shortcut=True,perturbationRadius=0.5)
    #TODO: maybe you should use planToConfig?
    print(motionplanning.getPlanJSONString())
    planner = robotplanning.planToConfig(world,robot,qtarget,edgeCheckResolution=1e-2,movingSubset=moving_joints, type='sbl', shortcut=True,perturbationRadius=0.3)
    if planner is None:
        return None
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

    numIters = 0
    t1 = time.time()
    print("Planning time,",numIters,"iterations",t1-t0)
    
    #to be nice to the C++ module, do this to free up memory
    plan.space.close()
    plan.close()
    return path


def optimizing_plan(world,robot,qtarget):
    """Plans for some number of iterations from the robot's current configuration to
    configuration qtarget.

    Returns None if no path was found, otherwise returns the best plan found.
    """
    #TODO: copy what's in feasible_plan, but change the way in which you to terminate
    t0 = time.time()

    moving_joints = [1,2,3,4,5,6,7]
    space = robotplanning.makeSpace(world=world,robot=robot,edgeCheckResolution=1e-2,movingSubset=moving_joints)
    plan = MotionPlan(space,type='prm')
    #TODO: maybe you should use planToConfig?
    
    settings = [dict(type='sbl',shortcut=True,perturbationRadius=0.5),
    dict(type='sbl',perturbationRadius=0.5,shortcut=True,restart=True,restartTermCond="{foundSolution:1,maxIters:100}"),
    dict(type="rrt*"),
    dict(type="lazyrrg*")
    ]
    
    filename = ["sbl", "sblrestart", "rrt", "lzyrrg"]
    config = robot.getConfig()
    
    for i in range(len(settings)):
        length = []
        success_rate = 0
        for j in range(10):
            robot.setConfig(config)
            planner = robotplanning.planToConfig(world,robot,qtarget,edgeCheckResolution=1e-2,movingSubset=moving_joints, **settings[i])
            print(planner)
            increment = 100                #there is a little overhead for each planMore call, so it is best not to set the increment too low
            t0 = time.time()
            while time.time() - t0 < 10:   #max 20 seconds of planning
                planner.planMore(increment)
                path = planner.getPath()
            if path and len(path)>0:
                success_rate += 0.1
                length.append(len(path))
            planner.close()   #frees a little memory... this is only really necessary if you are creating lots of planners
        with open(filename[i], "w") as f:
            f.write(str(length))
            f.write("success: {}, mean length: {}, std length: {}".format(str(success_rate), str(np.mean(length)), str(np.std(length))))
    return path

    numIters = 0
    t1 = time.time()
    print("Planning time,",numIters,"iterations",t1-t0)
    
    #to be nice to the C++ module, do this to free up memory
    plan.space.close()
    plan.close()
    return path

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
        return 0

    print("Planned path with length",trajectory.RobotTrajectory(robot,milestones=path).length())
    return 1
