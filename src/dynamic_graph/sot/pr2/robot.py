# -*- coding: utf-8 -*-
# Copyright 2013, Benjamin Coudrin, François Keith, LIRMM, CNRS
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST
#
# This file is part of dynamic-graph.
# dynamic-graph is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# dynamic-graph is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# dynamic-graph. If not, see <http://www.gnu.org/licenses/>.

#from dynamic_graph.sot.dynamics.abstract_robot import AbstractRobot
from dynamic_graph.sot.dynamics.humanoid_robot import AbstractHumanoidRobot
from dynamic_graph.ros.robot_model import RosRobotModel

class Pr2(AbstractHumanoidRobot):
    """
    This class instanciates a Pr2 robot.
    """

    OperationalPoints = ['right-wrist','left-wrist','waist','gaze','chest','left-ankle','r_gripper_joint']

    jointMap={}
    jointMap['BODY']      = 'base_link'
    jointMap['l_wrist']   = 'l_gripper_palm_link'
    jointMap['r_wrist']   = 'r_gripper_palm_link'
    jointMap['l_gripper'] = 'l_gripper_tool_frame'
    jointMap['r_gripper'] = 'r_gripper_tool_frame'
    jointMap['gaze']      = 'double_stereo_link'
    jointMap['torso']     = 'torso_lift_link'
    jointMap['l_ankle']   = 'base_link' # TODO?
    jointMap['r_gripper']   = 'r_gripper_l_finger_tip_link'

    tracedSignals = {
        'dynamic': ["com", "position", "velocity", "acceleration"],
        'device': ['control', 'state']
        }
        
    ignoredJoints = [
      # Remove the wheels
      'bl_caster_l_wheel_joint', 'bl_caster_r_wheel_joint', \
      'bl_caster_rotation_joint', 'br_caster_l_wheel_joint', \
      'br_caster_r_wheel_joint',  'br_caster_rotation_joint', \
      'fl_caster_l_wheel_joint',  'fl_caster_r_wheel_joint', \
      'fl_caster_rotation_joint', 'fr_caster_l_wheel_joint', \
      'fr_caster_r_wheel_joint',  'fr_caster_rotation_joint',\
      # Remove the laser, normally autonomous
      'laser_tilt_joint',         'laser_tilt_mount_joint',\

      # Remove the tip of the gripper
      'l_gripper_motor_accelerometer_joint', \
      'l_gripper_motor_screw_joint', \
      'l_gripper_motor_slider_joint'\
      'r_gripper_motor_accelerometer_joint', \
      'r_gripper_motor_screw_joint', \
      'r_gripper_motor_slider_joint']

    def specifySpecialLinks(self):
        for i in self.jointMap:
            self.dynamic.addJointMapping(i, self.jointMap[i])

        for i in self.ignoredJoints:
            self.dynamic.ignoreJoint(i)

    def __init__(self, name, device = None, tracer = None):
        AbstractHumanoidRobot.__init__ (self, name, tracer)
        self.device = device
        self.dynamic = RosRobotModel("{0}_dynamic".format(name))
        self.specifySpecialLinks()
        #Note: the param 'robot_description' should be defined before the next instruction
        self.dynamic.loadFromParameterServer()
        self.dimension = self.dynamic.getDimension()
        print self.dimension
        self.halfSitting = (0.,) * self.dimension
        lst = list(self.halfSitting)
        lst[12] = -0.33
        lst[14] = -0.47
        lst[25] = -0.33
        lst[27] = -0.47
        self.halfSitting = tuple(lst)

        # correct the initialization of the dynamic.
        self.dynamic.velocity.value = self.dimension*(0.,)
        self.dynamic.acceleration.value = self.dimension*(0.,)
        self.dynamic.ffposition.unplug()
        self.dynamic.ffvelocity.unplug()
        self.dynamic.ffacceleration.unplug()
        self.dynamic.setProperty('ComputeBackwardDynamics','true')
        self.dynamic.setProperty('ComputeAccelerationCoM','true')

        self.initializeRobot()

__all__ = ["Pr2"]
