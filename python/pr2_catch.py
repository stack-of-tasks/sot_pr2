# 1. Instanciate a Pr2
# The URDF description of the robot must have 
# been loaded in robot_description parameter
# on the Ros Parameter Server
from dynamic_graph.sot.pr2.robot import Pr2
from dynamic_graph.sot.core import RobotSimu
from dynamic_graph import plug
robot = Pr2('PR2', device=RobotSimu('PR2'))
plug(robot.device.state, robot.dynamic.position)

# 2. Ros binding
# roscore must be running
from dynamic_graph.ros import Ros
ros = Ros(robot)

# 3. Create a solver
from dynamic_graph.sot.application.velocity.precomputed_tasks import Solver
solver = Solver(robot)

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

# 5. Add a contact constraint with the robot and the floor
contact = MetaTaskKine6d('contact',robot.dynamic,'contact','left-ankle')
contact.feature.frame('desired')
contact.feature.selec.value = '011100'
contact.gain.setConstant(10)
contact.keep()
locals()['contactBase'] = contact

# 6. Push tasks in the solver
solver.push(taskRH.task)
solver.push(contactBase.task)

# Main loop
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
dt=1e-3
@loopInThread
def inc():
    robot.device.increment(dt)
    
runner=inc()
runner.once()
[go,stop,next,n]=loopShortcuts(runner)

print 'Type go to run the solver loop'