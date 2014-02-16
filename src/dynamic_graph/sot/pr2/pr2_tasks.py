from numpy import eye, array
from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.pr2.robot import *
from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d
from dynamic_graph.sot.dyninv import TaskInequality, TaskJointLimits
from dynamic_graph.sot.core.meta_task_visual_point import MetaTaskVisualPoint

from dynamic_graph.sot.application.velocity.precomputed_tasks import Solver, createCenterOfMassFeatureAndTask, createOperationalPointFeatureAndTask, initializeSignals


# 
def initialize (robot, solverType=SOT):
    """
    Initialize the solver, and define shortcuts for the operational points
    """

    # TODO: this should disappear in the end.     
    # --- center of mass ------------
    (robot.featureCom, robot.featureComDes, robot.comTask) = \
        createCenterOfMassFeatureAndTask(robot,
        '{0}_feature_com'.format(robot.name),
        '{0}_feature_ref_com'.format(robot.name),
        '{0}_task_com'.format(robot.name))

    # --- operational points tasks -----
    robot.features = dict()
    robot.tasks = dict()
    for op in robot.OperationalPoints:
        (robot.features[op], robot.tasks[op]) = \
            createOperationalPointFeatureAndTask(robot,
            op, '{0}_feature_{1}'.format(robot.name, op),
            '{0}_task_{1}'.format(robot.name, op))
        # define a member for each operational point
        w = op.split('-')
        memberName = w[0]
        for i in w[1:]:
            memberName += i.capitalize()
        setattr(robot, memberName, robot.features[op])

    initializeSignals (robot)

    # --- create solver --- #
    solver = Solver (robot, solverType)

    # --- push balance task --- #
    metaContact = Pr2ContactTask(robot)
    robot.tasks ['contact'] = metaContact.task
    robot.features ['contact'] = metaContact.feature
    metaContact.feature.selec.value = '011100'
    metaContact.featureDes.position.value = \
      array([[1,0,0,0],[0,1,0,0],[0,0,1,0.051],[0,0,0,1]])
    solver.push(robot.tasks['contact'])
#    solver.sot.addContact(robot.tasks['contact'])

    return solver


# -- HANDS ------------------------------------------------------------------
# TODO: directly use the feature.
def Pr2RightHandTask(robot):
    task=MetaTaskKine6d('right-wrist',robot.dynamic,'right-wrist','right-wrist')
    task.feature.frame('desired')
    return task

def Pr2LeftHandTask(robot):
    task=MetaTaskKine6d('left-wrist',robot.dynamic,'left-wrist','left-wrist')
    task.feature.frame('desired')
    return task

# -- CHEST ------------------------------------------------------------------
def Pr2ChestTask(robot):
    task=MetaTaskKine6d('chest',robot.dynamic,'chest','chest')
    task.feature.frame('desired')
    return task

def initPostureTask(robot):
  # --- TASK POSTURE --------------------------------------------------
  # set a default position for the joints.
  robot.features['featurePosition'] = FeaturePosture('featurePosition')
  plug(robot.device.state,robot.features['featurePosition'].state)
  robotDim = len(robot.dynamic.velocity.value)
  robot.features['featurePosition'].posture.value = robot.halfSitting

  postureTaskDofs = [True]*6 + [False]*(51-6)
  postureTaskDofs = [True]*(51)

  for dof,isEnabled in enumerate(postureTaskDofs):
    if dof > 6:
      robot.features['featurePosition'].selectDof(dof,isEnabled)

  robot.tasks['robot_task_position']=Task('robot_task_position')
  robot.tasks['robot_task_position'].add('featurePosition')
  # featurePosition.selec.value = toFlags((6,24))

  gainPosition = GainAdaptive('gainPosition')
  gainPosition.set(0.1,0.1,125e3)
  gainPosition.gain.value = 5
  plug(robot.tasks['robot_task_position'].error,gainPosition.error)
  plug(gainPosition.gain,robot.tasks['robot_task_position'].controlGain)



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
            "initialize", "Pr2ChestTask",
            "Pr2BaseTask", "initPostureTask"]
