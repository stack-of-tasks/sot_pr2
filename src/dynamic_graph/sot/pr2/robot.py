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

from dynamic_graph.sot.dynamics.abstract_robot import AbstractRobot
from dynamic_graph.ros.ros_sot_robot_model import RosSotRobotModel

class Pr2(AbstractRobot):
    """
    This class instanciates a Pr2 robot.
    """

    OperationalPoints = ['right-wrist','left-wrist','waist','gaze']

    tracedSignals = {
        'dynamic': ["com", "position", "velocity", "acceleration"],
        'device': ['control', 'state']
        }

    def __init__(self, name, device = None, tracer = None):
        AbstractRobot.__init__ (self, name, tracer)
        self.device = device
        self.dynamic = RosSotRobotModel("{0}_dynamic".format(name))
        self.dynamic.loadFromParameterServer()
        self.dimension = self.dynamic.getDimension()
        self.halfSitting = (0.,) * self.dimension
        self.initializeRobot()

__all__ = ["Pr2"]
