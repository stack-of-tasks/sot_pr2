from dynamic_graph import plug
from dynamic_graph.sot.pr2.pr2_tasks import *
from dynamic_graph.sot.application.velocity.precomputed_tasks import Solver
from dynamic_graph.ros import *
from time import sleep
plug(robot.device.state, robot.dynamic.position)
ros = Ros(robot)
sleep(2)
solver = initialize(robot)
robot.dynamic.velocity.value = robot.dimension*(0.,)
robot.dynamic.acceleration.value = robot.dimension*(0.,)
robot.dynamic.ffposition.unplug()
robot.dynamic.ffvelocity.unplug()
robot.dynamic.ffacceleration.unplug()
robot.dynamic.setProperty('ComputeBackwardDynamics','true')
robot.dynamic.setProperty('ComputeAccelerationCoM','true')
robot.device.control.unplug()
plug(solver.sot.control,robot.device.control)

dt = 0.001
taskRH = Pr2RightHandTask(robot)
taskLH = Pr2LeftHandTask(robot)
taskJL = Pr2JointLimitsTask(robot,dt)
taskContact = Pr2ContactTask(robot)
taskFov = Pr2FoVTask(robot,dt)
taskBase = Pr2BaseTask(robot)
rgrip = Pr2RightGripper(robot)
lgrip = Pr2LeftGripper(robot)

from dynamic_graph.sot.core.meta_tasks_kine import gotoNd
targetRH = (0.60,-0.2,0.8)
gotoNd(taskRH,targetRH,'111',(4.9,0.9,0.01,0.9))
targetLH = (0.65,0.3,0.6)
gotoNd(taskLH,targetLH,'111',(4.9,0.9,0.01,0.9))
taskFov.goto3D(targetRH)
targetBase = (0.2, -0.1, 0, 0, 0, 0.0)
gotoNd(taskBase,targetBase,'100011',(4.9,0.9,0.01,0.9))

rgrip.open()
lgrip.open()

solver.push(taskRH.task)
solver.push(taskLH.task)
solver.push(rgrip.task)
solver.push(lgrip.task)
solver.push(taskBase.task)
