# -*- coding: utf-8 -*-
"""
APT Motor Controller for Thorlabs.

Copyright (c) 2021, the qudi developers. See the AUTHORS.md file at the top-level directory of this
distribution and on <https://github.com/Ulm-IQO/qudi-iqo-modules/>

This file is part of qudi.

Qudi is free software: you can redistribute it and/or modify it under the terms of
the GNU Lesser General Public License as published by the Free Software Foundation,
either version 3 of the License, or (at your option) any later version.

Qudi is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with qudi.
If not, see <https://www.gnu.org/licenses/>.
"""

"""
This module was developed from PyAPT, written originally by Michael Leung
(mcleung@stanford.edu). Have a look in:
    https://github.com/HaeffnerLab/Haeffner-Lab-LabRAD-Tools/blob/master/cdllservers/APTMotor/APTMotorServer.py
APT.dll and APT.lib were provided to PyAPT thanks to SeanTanner@ThorLabs .
All the specific error and status code are taken from:
    https://github.com/UniNE-CHYN/thorpy
The rest of the documentation is based on the Thorlabs APT Server documentation
which can be obtained directly from
    https://www.thorlabs.com/software_pages/ViewSoftwarePage.cfm?Code=APT
"""

from collections import OrderedDict

from qudi.core.module import Base
from qudi.util.paths import get_home_dir
from qudi.util.paths import get_main_dir
from ctypes import c_long, c_buffer, c_float, windll, pointer
from qudi.interface.motor_interface import MotorInterface
import os
import platform

class KinesisMotor:
    pass

class KinesisStage(MotorInterface):
    """ Control class for an arbitrary collection of APTmotor axes.

    The required config file entries are based around a few key ideas:
      - There needs to be a list of axes, so that everything can be "iterated" across this list.
      - There are some config options for each axis that are all in sub-dictionary of the config file.
        The key is the axis label.
      - One of the config parameters is the constraints, which are given in a sub-sub-dictionary,
        which has the key 'constraints'.

    For example, a config file entry for a single-axis rotating half-wave-plate stage would look like:
    Example config for copy-paste

    hwp_motor:
        module.Class: 'motor.aptmotor.APTStage'
        options:
            dll_path:\ 'C:\Program Files\Thorlabs\'
            axis_labels:
                - phi
            phi:
                hw_type: 'TDC001'
                serial_num: 27500136
                pitch: 17.87
                unit: 'degree'
                constraints:
                    pos_min: -360
                    pos_max: 720
                    vel_min: 1.0
                    vel_max: 10.0
                    acc_min: 4.0
                    acc_max: 10.0

    A config file entry for a linear xy-axis stage would look like:

    hwp_motor:
        module.Class: 'motor.aptmotor.APTStage'
        options:
            dll_path: 'C:\\Program Files\\Thorlabs\\'
            axis_labels:
                - x
                - y
            x:
                hw_type: 'TDC001'
                serial_num: 00000000
                pitch:  1
                unit: 'm'
                constraints:
                    pos_min: 0
                    pos_max: 2
                    vel_min: 1.0
                    vel_max: 10.0
                    acc_min: 4.0
                    acc_max: 10.0
            y:
                hw_type: 'TDC001'
                serial_num: 00000001
                pitch: 1
                unit: 'm'
                constraints:
                    pos_min: -1
                    pos_max: 1
                    vel_min: 1.0
                    vel_max: 10.0
                    acc_min: 4.0
                    acc_max: 10.0

    """

    def get_constraints(self):
        """ Retrieve the hardware constrains from the motor device.

        @return dict: dict with constraints for the magnet hardware. These
                      constraints will be passed via the logic to the GUI so
                      that proper display elements with boundary conditions
                      could be made.

        Provides all the constraints for each axis of a motorized stage
        (like total travel distance, velocity, ...)
        Each axis has its own dictionary, where the label is used as the
        identifier throughout the whole module. The dictionaries for each axis
        are again grouped together in a constraints dictionary in the form

            {'<label_axis0>': axis0 }

        where axis0 is again a dict with the possible values defined below. The
        possible keys in the constraint are defined here in the interface file.
        If the hardware does not support the values for the constraints, then
        insert just None. If you are not sure about the meaning, look in other
        hardware files to get an impression.

        Example of how a return dict with constraints might look like:
        ==============================================================

        constraints = {}

        axis0 = {}
        axis0['label'] = 'x'    # it is very crucial that this label coincides
                                # with the label set in the config.
        axis0['unit'] = 'm'     # the SI units, only possible m or degree
        axis0['ramp'] = ['Sinus','Linear'], # a possible list of ramps
        axis0['pos_min'] = 0,
        axis0['pos_max'] = 100,  # that is basically the traveling range
        axis0['pos_step'] = 100,
        axis0['vel_min'] = 0,
        axis0['vel_max'] = 100,
        axis0['vel_step'] = 0.01,
        axis0['acc_min'] = 0.1
        axis0['acc_max'] = 0.0
        axis0['acc_step'] = 0.0

        axis1 = {}
        axis1['label'] = 'phi'   that axis label should be obtained from config
        axis1['unit'] = 'degree'        # the SI units
        axis1['ramp'] = ['Sinus','Trapez'], # a possible list of ramps
        axis1['pos_min'] = 0,
        axis1['pos_max'] = 360,  # that is basically the traveling range
        axis1['pos_step'] = 100,
        axis1['vel_min'] = 1,
        axis1['vel_max'] = 20,
        axis1['vel_step'] = 0.1,
        axis1['acc_min'] = None
        axis1['acc_max'] = None
        axis1['acc_step'] = None

        # assign the parameter container for x to a name which will identify it
        constraints[axis0['label']] = axis0
        constraints[axis1['label']] = axis1
        """
        pass

    def move_rel(self,  param_dict):
        """ Moves stage in given direction (relative movement)

        @param dict param_dict: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                 {'axis_label': <the-abs-pos-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.

        A smart idea would be to ask the position after the movement.

        @return int: error code (0:OK, -1:error)
        """
        pass

    def move_abs(self, param_dict):
        """ Moves stage to absolute position (absolute movement)

        @param dict param_dict: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                 {'axis_label': <the-abs-pos-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.

        @return int: error code (0:OK, -1:error)
        """
        pass

    @abstractmethod
    def abort(self):
        """ Stops movement of the stage

        @return int: error code (0:OK, -1:error)
        """
        pass

    def get_pos(self, param_list=None):
        """ Gets current position of the stage arms

        @param list param_list: optional, if a specific position of an axis
                                is desired, then the labels of the needed
                                axis should be passed in the param_list.
                                If nothing is passed, then from each axis the
                                position is asked.

        @return dict: with keys being the axis labels and item the current
                      position.
        """
        pass

    def get_status(self, param_list=None):
        """ Get the status of the position

        @param list param_list: optional, if a specific status of an axis
                                is desired, then the labels of the needed
                                axis should be passed in the param_list.
                                If nothing is passed, then from each axis the
                                status is asked.

        @return dict: with the axis label as key and the status number as item.
        """
        pass

    def calibrate(self, param_list=None):
        """ Calibrates the stage.

        @param dict param_list: param_list: optional, if a specific calibration
                                of an axis is desired, then the labels of the
                                needed axis should be passed in the param_list.
                                If nothing is passed, then all connected axis
                                will be calibrated.

        @return int: error code (0:OK, -1:error)

        After calibration the stage moves to home position which will be the
        zero point for the passed axis. The calibration procedure will be
        different for each stage.
        """
        pass

    def get_velocity(self, param_list=None):
        """ Gets the current velocity for all connected axes.

        @param dict param_list: optional, if a specific velocity of an axis
                                is desired, then the labels of the needed
                                axis should be passed as the param_list.
                                If nothing is passed, then from each axis the
                                velocity is asked.

        @return dict : with the axis label as key and the velocity as item.
        """
        pass

    def set_velocity(self, param_dict):
        """ Write new value for velocity.

        @param dict param_dict: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                 {'axis_label': <the-velocity-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.

        @return int: error code (0:OK, -1:error)
        """
        pass
