from klampt import *
from klampt import vis
from klampt.math import vectorops,so3,se3
from klampt.model.trajectory import Trajectory
from klampt.io import numpy_convert


def main():
    # load the robot / world file
    fn = "world.xml"
    world = WorldModel()
    res = world.readFile(fn)
    if not res:
        print("Unable to read file", fn)
        exit(0)

    robot = world.robot(0)

    vis.createWindow()
    # closeup_viewport = {'up': {'z': 0, 'y': 1, 'x': 0}, 'target': {'z': 0, 'y': 0, 'x': 0}, 'near': 0.1, 'position': {'z': 1.0, 'y': 0.5, 'x': 0.0}, 'far': 1000}
    # vis.setViewport(closeup_viewport)
    vis.add("world", world)
    vis.loop()

main()