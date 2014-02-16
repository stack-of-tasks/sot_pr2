# 0. TRICK: import Dynamic as the first command to avoid the crash at the exit
from dynamic_graph.sot.dynamics import Dynamic

# 1. Instanciate a Pr2
# The URDF description of the robot must have 
# been loaded in robot_description parameter
# on the Ros Parameter Server
# 1. Init robot, ros binding, solver
from dynamic_graph.sot.pr2.pr2_tasks import *
from dynamic_graph.sot.pr2.robot import *
from dynamic_graph.sot.core import RobotSimu
from dynamic_graph import plug
robot = Pr2('PR2', device=RobotSimu('PR2'))
plug(robot.device.state, robot.dynamic.position)

# 2. Ros binding
# roscore must be running
from dynamic_graph.ros import Ros
ros = Ros(robot)

# Use kine solver (with inequalities)
solver = initialize(robot)

# 4. Define a position task for the right hand
from dynamic_graph.sot.core.meta_tasks_kine import gotoNd, MetaTaskKine6d
from numpy import eye
from dynamic_graph.sot.core.matrix_util import matrixToTuple
taskRH=MetaTaskKine6d('rh',robot.dynamic,'rh','right-wrist')
Pr2handMgrip = eye(4); Pr2handMgrip[0:3,3] = (0.18,0,0)
taskRH.opmodif = matrixToTuple(Pr2handMgrip)
taskRH.feature.frame('desired')
targetR=(0.65,0.2,0.9)
selec='111'
gain=(4.9,0.9,0.01,0.9)
gotoNd(taskRH,targetR,selec,gain)

# 6. Push tasks in the solver
solver.push(taskRH.task)

# Main loop
dt=3e-3
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
@loopInThread
def inc():
    robot.device.increment(dt)
    
runner=inc()
runner.once()
[go,stop,next,n]=loopShortcuts(runner)

print 'Type go to run the solver loop'

