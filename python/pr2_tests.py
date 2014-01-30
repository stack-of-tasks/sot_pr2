#This simple test allows to check the joints of the robot one by one.
# 

# 1. Init robot, ros binding, solver
from dynamic_graph.sot.pr2.pr2_tasks import *
from dynamic_graph.sot.pr2.robot import *
from dynamic_graph.sot.core.robot_simu import RobotSimu
from dynamic_graph import plug

robot = Pr2('pr2', device=RobotSimu('pr2'))
plug(robot.device.state, robot.dynamic.position)

from dynamic_graph.ros import *
ros = Ros(robot)

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
initPostureTask(robot)

solver.push(robot.tasks['robot_task_position'])

# allows to change only one joint and to test 
def testJoint(robot, index, angle):
  robot.features['featurePosition'].posture.value = (0,)*index + (angle,) + (0,)*(51-index)
  robot.dynamic.signal('chest').recompute(robot.device.state.time)
  robot.dynamic.signal('right-wrist').recompute(robot.device.state.time)
  print robot.dynamic.signal('chest').value
  print robot.dynamic.signal('right-wrist').value

