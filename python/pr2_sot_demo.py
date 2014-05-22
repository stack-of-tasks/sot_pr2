# 0. TRICK: import Dynamic as the first command to avoid the crash at the exit
from dynamic_graph.sot.dynamics import Dynamic

# 1. Init robot, ros binding, solver
from dynamic_graph.sot.pr2.pr2_tasks import *
from dynamic_graph.sot.pr2.robot import *
from dynamic_graph.sot.core.robot_simu import RobotSimu
from dynamic_graph import plug

# creates the robot.
robot = Pr2('PR2', device=RobotSimu('PR2'))
plug(robot.device.state, robot.dynamic.position)

# publish to ros
from dynamic_graph.ros import *
ros = Ros(robot)

# Use kine solver (with inequalities)
from dynamic_graph.sot.dyninv import SolverKine
solver = initialize(robot, SolverKine)


# 2. Main loop
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
dt=3e-3
@loopInThread
def inc():
    robot.device.increment(dt)

runner=inc()
[go,stop,next,n]=loopShortcuts(runner)

# 3. Init Tasks
taskRH = Pr2RightHandTask(robot)
taskLH = Pr2LeftHandTask(robot)
taskJL = Pr2JointLimitsTask(robot,dt)
taskFov = Pr2FoVTask(robot,dt)
taskBase = Pr2BaseTask(robot)
taskChest = Pr2ChestTask(robot)
(taskWeight, featureWeight) = Pr2Weight(robot)
initPostureTask(robot)

# 4. Formulate problem
from dynamic_graph.sot.core.meta_tasks_kine import gotoNd

# 4.0 chest
gotoNd(taskChest,(-0.05,0.0,1),'111',(4.9,0.9,0.01,0.9))

# 4.1 Right hand
targetRH = (0.60,-0.2,0.8)
gotoNd(taskRH,targetRH,'111',(4.9,0.9,0.01,0.9))

# 4.2 Left hand
targetLH = (0.65,0.2,0.8)
gotoNd(taskLH,targetLH,'111',(4.9,0.9,0.01,0.9))

# 4.3 Look at the right hand target
taskFov.goto3D(targetRH)

# 4.4 Base position
targetBase = (0,0,0,0,0,0)
gotoNd(taskBase,targetBase,'100011',(4.9,0.9,0.01,0.9))


# 4.4 Base position
solver.push(taskJL)
solver.push(taskFov.task)
solver.push(taskBase.task)
solver.push(taskRH.task)
solver.push(taskWeight)

#solver.push(taskChest.task)

print ('Type go to run the solver loop')

