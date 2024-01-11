# -*- coding: utf-8 -*-

"""
This file contains the general logic for step motor.

Qudi is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Qudi is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Qudi. If not, see <http://www.gnu.org/licenses/>.

Copyright (c) the Qudi Developers. See the COPYRIGHT.txt file at the
top-level directory of this distribution and at <https://github.com/Ulm-IQO/qudi/>
"""

import datetime
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from PySide2 import QtCore
from qudi.core.connector import Connector
from qudi.core.configoption import ConfigOption
from qudi.core.module import LogicBase
from qudi.util.mutex import Mutex


class KinesisMotorLogic(LogicBase):
    _kinesis_motor = Connector(interface='MotorInterface')

    _kinesis_motor_X = ConfigOption(name='x_serial_num')
    _kinesis_motor_Y = ConfigOption(name='y_serial_num')
    _kinesis_motor_Z = ConfigOption(name='z_serial_num')

    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)
        self.thread_lock = Mutex()

    def on_activate(self):
        self._motor = self._kinesis_motor()

    def on_deactivate(self):
        return 0

    def get_constraints(self):
        return self._motor.get_constraints()

    def move_rel(self, motor_channel=None, degree=None):
        return self._motor.move_rel(motor_channel, degree)

    def move_abs(self, motor_channel=None, degree=None):
        return self._motor.move_abs(motor_channel, degree)

    def get_pos(self, motor_channel=None, degree=None):
        return self._motor.get_pos(motor_channel, degree)

    def get_status(self, motor_channel=None, degree=None):
        return self._motor.get_status(motor_channel, degree)

    def calibrate(self, motor_channel=None, degree=None):
        return self._motor.calibrate(motor_channel, degree)

    def get_velocity(self, motor_channel=None, degree=None):
        return self._motor.get_velocity(motor_channel, degree)

