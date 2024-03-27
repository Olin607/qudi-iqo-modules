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
from qudi.hardware.scanner import Scanner


class KinesisMotorLogic(LogicBase):
    kinesis_motor = Connector(interface='MotorInterface')

    # Set up all the configurations for the x,y,z motors.
    _x = ConfigOption('x_serial_num', '27262884')
    _y = ConfigOption('y_serial_num', '27256199')
    _z = ConfigOption('z_serial_num', '27256522')

    # Constraints for the x,y and z motors.
    _pos_min = ConfigOption('pos_min', 0)
    _pos_max = ConfigOption('pos_max', .025)
    _vel_min = ConfigOption('vel_min', .001)
    _vel_max = ConfigOption('vel_max', .015)
    _acc_min = ConfigOption('acc_min', .004)
    _acc_max = ConfigOption('acc_max', .01)

    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)
        self.thread_lock = Mutex()
        self.scanner = None

    def get_x_serial_number(self):
        return self._x

    def get_y_serial_number(self):
        return self._y

    def get_z_serial_number(self):
        return self._z

    def get_position_min(self):
        return self._pos_min

    def get_position_max(self):
        return self._pos_max

    def get_velocity_min(self):
        return self._vel_min

    def get_velocity_max(self):
        return self._vel_max

    def get_acceleration_min(self):
        return self._acc_min

    def get_acceleration_max(self):
        return self._acc_max

    def on_activate(self):
        self._motor = self.kinesis_motor()
        self.scanner = Scanner(self._motor.x_motor, self._motor.y_motor, self._motor.z_motor)
        self.run_raster()

    def run_raster(self):
        answer = input('Run remote control? (yes/no): ').lower()
        if answer == 'yes':
            self.scanner.query_raster()
            self.scanner.raster()
        else:
            print("Remote control not initiated.")

    def on_deactivate(self):
        return self._motor.on_deactivate()

    def get_constraints(self):
        return self._motor.get_constraints()

    def move_rel(self, param_dict=None):
        if param_dict is None:
            self.log.error("Move_rel's param_dict is empty.")
            return -1
        return self._motor.move_rel(param_dict)

    def move_abs(self, param_dict=None):
        if param_dict is None:
            self.log.error("Move_abs's param_dict is empty.")
            return -1
        return self._motor.move_abs(param_dict)

    def get_pos(self, param_list= None):
        if param_list is None:
            self.log.error("get_pos's param_list is empty.")
            return -1
        return self._motor.get_pos(param_list)

    def get_status(self, param_list):
        if param_list is None:
            self.log.error("get_status's param_list is empty.")
            return -1
        return self._motor.get_status(param_list)

    def calibrate(self, param_dict=None):
        if param_dict is None:
            self.log.error("calibrate param_dict is empty.")
            return -1
        return self._motor.calibrate(param_dict)

    def get_velocity(self, param_dict=None):
        if param_dict is None:
            self.log.error("get_velocity's param_dict is empty.")
            return -1
        return self._motor.get_velocity(param_dict)

    def set_velocity(self, param_dict=None):
        if param_dict is None:
            self.log.error("set_velocity's param_list is empty.")
            return -1
        return self._motor.set_velocity(param_dict)
