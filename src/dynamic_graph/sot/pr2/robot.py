# -*- coding: utf-8 -*-
# Copyright 2013, Benjamin Coudrin, LIRMM, CNRS
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

    OperationalPoints = ['right-wrist','left-wrist','waist','gaze','chest','left-ankle']
    
    SpecialLinks  = ['BODY', 'l_wrist', 'r_wrist', 'l_gripper', 'r_gripper', 'gaze','torso','l_ankle']
    #SpecialNames = ['base_link', 'l_wrist_roll_link', 'r_wrist_roll_link', 'l_gripper_palm_link', 'r_gripper_palm_link', 'double_stereo_link','torso_lift_link','base_link']
    SpecialNames = ['base_link', 'l_wrist_roll_link', 'base_link', 'l_gripper_palm_link', 'r_gripper_palm_link', 'double_stereo_link','torso_lift_link','base_link']

    tracedSignals = {
        'dynamic': ["com", "position", "velocity", "acceleration"],
        'device': ['control', 'state']
        }
        
    def specifySpecialLinks(self):
        if len(self.SpecialLinks) == len(self.SpecialNames):
            for i in range(0,len(self.SpecialLinks)):
                self.dynamic.addJointMapping(self.SpecialLinks[i], self.SpecialNames[i])
        else:
            print 'No Special joints added : SpecialLinks.size != SpecialJoints.size'

    def __init__(self, name, device = None, tracer = None):
        AbstractHumanoidRobot.__init__ (self, name, tracer)
        self.device = device
        self.dynamic = RosRobotModel("{0}_dynamic".format(name))
        self.specifySpecialLinks()
        #Note: the param 'robot_description' should be defined before the next instruction
        self.dynamic.loadFromParameterServer()
        self.dimension = self.dynamic.getDimension()
        self.halfSitting = (0.,) * self.dimension
        self.initializeRobot()

__all__ = ["Pr2"]
