from numpy import eye, array
from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.pr2.robot import *
from dynamic_graph.sot.dyninv import SolverKine
from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d
from dynamic_graph.sot.dyninv import TaskInequality, TaskJointLimits
from dynamic_graph.sot.core.meta_task_visual_point import MetaTaskVisualPoint
from dynamic_graph.ros import *

from dynamic_graph.sot.application.velocity.precomputed_tasks import Solver



def initPr2RosSimuProblem():
    robot = Pr2('pr2', device=RobotSimu('pr2'))
    plug(robot.device.state, robot.dynamic.position)
    ros = Ros(robot)
    solver = Solver(robot, SolverKine)
    return [robot,ros,solver]
    
def initPr2RosProblem():
    plug(robot.device.state, robot.dynamic.position)
    ros = Ros(robot)
    solver = Solver(robot, SolverKine)
    return [robot,ros,solver]


# -- HANDS ------------------------------------------------------------------
    
Pr2handMgrip = eye(4)
Pr2handMgrip[0:3,3] = (0.18,0,0)

def Pr2RightHandTask(robot):
    task=MetaTaskKine6d('rh',robot.dynamic,'rh','right-wrist')
    task.opmodif = matrixToTuple(Pr2handMgrip)
    task.feature.frame('desired')
    return task
    
def Pr2LeftHandTask(robot):
    task=MetaTaskKine6d('lh',robot.dynamic,'lh','left-wrist')
    task.opmodif = matrixToTuple(Pr2handMgrip)
    task.feature.frame('desired')
    return task


# -- CAMS  ------------------------------------------------------------------

Pr2headMcam = array([[0.0,0.0,1.0,0.0],[1.,0.0,0.0,0.0],[0.0,1.,0.0,0.0],[0.0,0.0,0.0,1.0]])
    
def Pr2GazeTask(robot):
    task = MetaTaskVisualPoint('gaze',robot.dynamic,'head','gaze')
    task.opmodif = matrixToTuple(Pr2headMcam)
    return task

def Pr2FoVTask(robot,dt):
    task = MetaTaskVisualPoint('FoV',robot.dynamic,'head','gaze')
    task.opmodif = matrixToTuple(Pr2headMcam)
    task.task=TaskInequality('taskFoVineq')
    task.task.add(task.feature.name)
    [Xmax,Ymax]=[0.38,0.28]
    task.task.referenceInf.value = (-Xmax,-Ymax)    # Xmin, Ymin
    task.task.referenceSup.value = (Xmax,Ymax)  # Xmax, Ymax
    task.task.dt.value=dt
    task.task.controlGain.value=0.01
    task.featureDes.xy.value = (0,0)
    return task
    

# -- JOINTS LIMITS  ---------------------------------------------------------

def Pr2JointLimitsTask(robot,dt):
    robot.dynamic.upperJl.recompute(0)
    robot.dynamic.lowerJl.recompute(0)
    task = TaskJointLimits('taskJL')
    plug(robot.dynamic.position,task.position)
    task.controlGain.value = 10
    task.referenceInf.value = robot.dynamic.lowerJl.value
    task.referenceSup.value = robot.dynamic.upperJl.value
    task.dt.value = dt
    task.selec.value = toFlags(range(18,25)+range(26,27)+range(28,31)+range(32,40)+range(41,42)+range(43,46)+range(47,50))
    return task
    

# -- CONTACT  --------------------------------------------------------------- 

def Pr2ContactTask(robot):
    task = MetaTaskKine6d('contact',robot.dynamic,'contact','left-ankle')
    task.feature.frame('desired')
    task.feature.selec.value = '011100'
    task.gain.setConstant(10)
    #locals()['contact'] = task
    return task
    
    
def Pr2FixedContactTask(robot):
    task = MetaTaskKine6d('contactFixed',robot.dynamic,'contact','left-ankle')
    task.feature.frame('desired')
    task.gain.setConstant(10)
    #task.feature.selec.value = '111111'
    #locals()['contactFixed'] = task
    return task


# -- WAIST  ------------------------------------------------------------------ 

baseMground=eye(4);
baseMground[0:3,3] = (0,0,-0.051)

def Pr2BaseTask(robot):
    task = MetaTaskKine6d('base',robot.dynamic,'waist','waist')
    task.opmodif = matrixToTuple(baseMground)
    task.feature.frame('desired')
    task.feature.selec.value = '011100'
    task.gain.setConstant(10)
    return task

    
__all__ = ["Pr2RightHandTask", "Pr2LeftHandTask", "Pr2GazeTask",
            "Pr2FoVTask", "Pr2JointLimitsTask", "Pr2ContactTask",
            "Pr2FixedContactTask", "initPr2RosSimuProblem",
            "Pr2BaseTask", "initPr2RosProblem"]
