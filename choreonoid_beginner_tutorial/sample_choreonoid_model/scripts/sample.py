#!/usr/bin/env python

# export PYTHONPATH=`pkg-config choreonoid --variable plugindir`/python:$PYTHONPATH
from cnoid import Body

bodyLoader = Body.BodyLoader()
robot = bodyLoader.load("/opt/ros/melodic/share/openhrp3/share/OpenHRP-3.1/sample/model/sample1.wrl")
if not robot:
    print("failed to load model")
    exit(1)

for i in range(robot.numJoints()):
    print(robot.joint(i).name())
