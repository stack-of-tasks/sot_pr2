# -*- coding: utf-8 -*-
# Copyright 2013, Benjamin Coudrin, LIRMM, LAAS, CNRS
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

print("Prologue PR2")

from dynamic_graph.entity import PyEntityFactoryClass
from dynamic_graph.sot.pr2.robot import Pr2

Device = PyEntityFactoryClass('Pr2Device')

robot = Pr2(name = 'robot', device = Device('PR2'))

#todo: necessary?
#plug(robot.device.state, robot.dynamic.position)

# Make sure only robot is visible from the outside.
__all__ = ["robot"]

