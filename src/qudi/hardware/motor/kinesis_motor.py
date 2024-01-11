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
    https://github.com/HaeffnerLab/Haeffner-Lab-LabRAD-Tools/blob/master/cdllservers/KinesisMotor/KinesisMotorServer.py
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
from abc import abstractmethod
from pylablib.devices import Thorlabs
from qudi.core.configoption import ConfigOption
import os
import platform
import pylablib
import logging

log = logging.getLogger(__name__)

class KinesisMotor:
    def __init__(self, label):
        self.label = label

class KinesisStage(MotorInterface):
    """ Control class for an arbitrary collection of KinesisMotor axes.

    The required config file entries are based around a few key ideas:
      - There needs to be a list of axes, so that everything can be "iterated" across this list.
      - There are some config options for each axis that are all in sub-dictionary of the config file.
        The key is the axis label.
      - One of the config parameters is the constraints, which are given in a sub-sub-dictionary,
        which has the key 'constraints'.

    For example, a config file entry for a single-axis rotating half-wave-plate stage would look like:
    Example config for copy-paste

    hwp_motor:
        module.Class: 'motor.KinesisMotor.APTStage'
        options:
            dll_path:\ 'C:\Program Files\Thorlabs\'
            axis_labels:
                - phi
            phi:
                hw_type: 'qTDC001'
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
        module.Class: 'motor.KinesisMotor.APTStage'
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

    # Grab all the configurations for the x,y,z motors in the olin file.
    _x_serial_number = ConfigOption(name='x_serial_num')
    _y_serial_number = ConfigOption(name='y_serial_num')
    _z_serial_number = ConfigOption(name='z_serial_num')
    _position_min = ConfigOption(name='pos_min')
    _position_max = ConfigOption(name='pos_max')
    _velocity_min = ConfigOption(name='vel_min')
    _velocity_max = ConfigOption(name='vel_max')
    _acceleration_min = ConfigOption(name='acc_min')
    _acceleration_max = ConfigOption(name='acc_max')

    def on_activate(self):
        """ Initialize instance variables and connect to hardware as configured.
        """
        self.log.warning("This module has not been tested on the new qudi core."
                         "Use with caution and contribute bug fixed back, please.")
        self.X_motor = Thorlabs.KinesisMotor(self._x_serial_number)
        self.Y_motor = Thorlabs.KinesisMotor(self._y_serial_number)
        self.Z_motor = Thorlabs.KinesisMotor(self._z_serial_number)

        # # create the magnet dump folder
        # # TODO: Magnet stuff needs to move to magnet interfuses. It cannot be in the motor stage class.
        # self._magnet_dump_folder = self._get_magnet_dump()
        #
        # # Path to the Thorlabs KinesisMotor DLL
        # # Check the config for the DLL path first
        # if 'dll_path' in config:
        #     path_dll = config['dll_path']
        #
        # # Otherwise, look in the "standard form" thirdparty directory
        # else:
        #
        #     if platform.architecture()[0] == '64bit':
        #         path_dll = os.path.join(get_main_dir(),
        #                                 'thirdparty',
        #                                 'thorlabs',
        #                                 'win64',
        #                                 'APT.dll')
        #     elif platform.architecture()[0] == '32bit':
        #         path_dll = os.path.join(get_main_dir(),
        #                                 'thirdparty',
        #                                 'thorlabs',
        #                                 'win64',
        #                                 'APT.dll')
        #     else:
        #         self.log.error('Unknown platform, cannot load the Thorlabs dll.')
        #
        # # Get the list of axis labels.
        # if 'axis_labels' in config.keys():
        #     axis_label_list = config['axis_labels']
        # else:
        #     self.log.error(
        #         'No axis labels were specified for the KinesisMotor stage.'
        #         'It is impossible to proceed. You might need to read more about how to configure the APTStage'
        #         'in the config file, and you can find this information (with example) at'
        #         'https://ulm-iqo.github.io/qudi-generated-docs/html-docs/classKinesisMotor_1_1APTStage.html#details'
        #     )
        #
        # # The references to the different axis are stored in this dictionary:
        # self._axis_dict = OrderedDict()
        #
        # hw_conf_dict = self._get_config()
        #
        # limits_dict = self.get_constraints()
        #
        # for axis_label in axis_label_list:
        #     serialnumber = hw_conf_dict[axis_label]['serial_num']
        #     hw_type = hw_conf_dict[axis_label]['hw_type']
        #     label = axis_label
        #     pitch = hw_conf_dict[axis_label]['pitch']
        #     unit = hw_conf_dict[axis_label]['unit']
        #
        #     self._axis_dict[axis_label] = KinesisMotor(path_dll,
        #                                            serialnumber,
        #                                            hw_type,
        #                                            label,
        #                                            unit
        #                                            )
        #     self._axis_dict[axis_label].initializeHardwareDevice()
        #
        #     # adapt the hardware controller to the proper unit set:
        #     if hw_conf_dict[axis_label]['unit'] == '°' or hw_conf_dict[axis_label]['unit'] == 'degree':
        #         unit = 2  # for rotation movement
        #         # FIXME: the backlash parameter has to be taken from the config and
        #         #       should not be hardcoded here!!
        #         backlash_correction = 0.2
        #     else:
        #         unit = 1  # default value for linear movement
        #         backlash_correction = 0.10e-3
        #
        #     self._axis_dict[axis_label].set_stage_axis_info(
        #         limits_dict[axis_label]['pos_min'],
        #         limits_dict[axis_label]['pos_max'],
        #         pitch=pitch,
        #         unit=unit
        #     )
        #     self._axis_dict[axis_label].setVelocityParameters(
        #         limits_dict[axis_label]['vel_min'],
        #         limits_dict[axis_label]['acc_max'],
        #         limits_dict[axis_label]['vel_max']
        #     )
        #
        #     self._axis_dict[axis_label].set_velocity(limits_dict[axis_label]['vel_max'])
        #
        #     # TODO: what does this do?
        #     self._axis_dict[axis_label].setHardwareLimitSwitches(2, 2)
        #
        #     self._axis_dict[axis_label]._wait_until_done = False
        #
        #     # set the backlash correction since the forward movement is
        #     # preciser than the backward:
        #     self._axis_dict[axis_label].set_backlash(backlash_correction)

    def on_deactivate(self):
        self.X_motor.close()
        self.Y_motor.close()
        self.Z_motor.close()
        log.debug("X, Y, and Z motors disconnected")

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
        self.constraints = {}

        self.constraints['position_min'] = self._position_min
        self.constraints['position_max'] = self._position_max
        self.constraints['velocity_min'] = self._velocity_min
        self.constraints['velocity_max'] = self._velocity_max
        self.constraints['acceleration_min'] = self._acceleration_min
        self.constraints['acceleration_max'] = self._acceleration_max

        return self.constraints

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
        constraints = self.get_constraints()
        curr_pos_dict = self.get_pos()

        self.X_motor.move_by(param_dict['x'])
        self.Y_motor.move_by(param_dict['y'])
        self.Z_motor.move_by(param_dict['z'])

    def move_abs(self, param_dict):
        """ Moves stage to absolute position (absolute movement)

        @param dict param_dict: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                 {'axis_label': <the-abs-pos-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.

        @return int: error code (0:OK, -1:error)
        """
        constraints = self.get_constraints()
        curr_pos_dict = self.get_pos()

        self.X_motor.move_to(param_dict['x'])
        self.Y_motor.move_to(param_dict['y'])
        self.Z_motor.move_to(param_dict['z'])

    def abort(self):
        """ Stops movement of the stage

        @return int: error code (0:OK, -1:error)
        """
        self.X_motor.abort()
        self.Y_motor.abort()
        self.Z_motor.abort()
        log.debug('X, Y, and Z motor were all aborted.')

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
        position = {}

        position['x'] = self.X_motor.get_position(param_list['x'])
        position['y'] = self.Y_motor.get_position(param_list['y'])
        position['z'] = self.Z_motor.get_position(param_list['z'])

        return position

    def get_status(self, param_list=None):
        """ Get the status of the position

        @param list param_list: optional, if a specific status of an axis
                                is desired, then the labels of the needed
                                axis should be passed in the param_list.
                                If nothing is passed, then from each axis the
                                status is asked.

        @return dict: with the axis label as key and the status number as item.
        """
        return self.X_motor.get_status() or self.Y_motor.get_status() or self.Z_motor.get_status()

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
        raise InterfaceImplementationError('MagnetStageInterface>calibrate')

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

    def set_velocity(self, param_dict=None):
        """ Write new value for velocity.

        @param dict param_dict: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                 {'axis_label': <the-velocity-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.

        @return int: error code (0:OK, -1:error)
        """
        pass

    def wait_move(self, query_interval=1e-3):
        """Repeatedly query whether the stage is moving by some time interval in seconds.

        Parameters
        ----------
        query_interval : float
            How many seconds to wait before the next stage movement query.
            Smaller intervals improve time-sensitive responses, but might tie up other system
            resources more agressively. The default query time checks for stage movement
            once every millisecond (1e-3 s).

        motor.wait_move() supplied by pylablib.
        """
        while self.is_moving():
            time.sleep(query_interval)

    def query_raster(self, user_scale='mm', unit_conv=1e-3):
        print('Welcome to interactive mode. All or some of the command-line\n'
              'arguments have not been detected. This script will perform a\n'
              'measurement at each coordinate of a grid, specified by the\n'
              'start position, end position, and number of subdivisions in\n'
              'three dimensions.\n')
        print(f'Enter absolute starting position in {user_scale} '
              'or leave blank to use current position.')
        start_query = input('(x_start, y_start, z_start): ')
        if start_query == '':
            start_pos = self.get_position()
        else:
            start_pos = tuple(unit_conv * val for val in map(float, start_query.split(',')))

        print(f'Enter absolute stopping position in {user_scale} '
              'or leave blank to use current position.')
        stop_query = input('(x_stop, y_stop, z_stop): ')
        if stop_query == '':
            stop_pos = self.get_position()
        else:
            stop_pos = tuple(unit_conv * val for val in map(float, stop_query.split(',')))

        print('Enter number of steps or leave blank for single measurement: ')
        num_query = input('(x_steps, y_steps, z_steps): ')
        if num_query == '':
            num_steps = (1, 1, 1)
        else:
            num_steps = tuple(map(int, num_query.split(',')))

        print(f'\nThe following scan will be conducted:\n'
              f'start_pos\t=\t{start_pos} m\n'
              f'stop_pos\t=\t{stop_pos} m\n'
              f'num_steps\t=\t{num_steps}\n'
              f'Total measurements\t{np.product(num_steps)}\n')

        answer = input('Continue? ([Y]/n): ')
        if answer.lower() not in ['', 'y', 'yes']:
            print('Aborting script')
            exit()
        return (start_pos, stop_pos, num_steps)

    def raster(self, start_pos, stop_pos, num_steps, callback=None, reset=False):
        """Move the stage from start to stop in a specified number of steps.

        Parameters
        ----------
        start_pos : array-like
            Absolute starting coordinate (x, y, z), specified in meters.
        stop_pos : array-like
            Absolute stopping coordinate (x, y, z), specified in meters.
        num_steps: array-like of integers
            The number of steps to make in each (x, y, z) direction.
            Each of x_steps, y_steps, and z_steps must be >=1.
        callback : function, optional
            A callable process to perform at each coordinate of the raster grid.
            Any return value from the callback function is lumped into an array
            and presented as a return value of the raster function.
        reset : bool, optional
            Whether to move back to the starting position when complete
            (default=False).

        Returns
        -------
        Array or None
            An array of values obtained by the callback function if the
            callback function returns a value; None otherwise.

            Shape of returned array: (z_steps, y_steps, x_steps, c_return),
            where c_return is the shape of the return value from the callback
            function.

        Notes
        -----
        The visited coordinates are subsequently stored as an instance attribute
        and accessible as self.coords after execution of this function. The
        value stored in self.coords corresponds to the coordinates of the most
        recent run since this parent object has been created.

        start_pos and stop_pos specify opposite verticies of a 3D cube, and
        movement is swept through the x-axis first, followed by the y-axis, and
        finally the z-axis. The step size in each dimension is automatically
        determined based on the desired number of totals steps specified in the
        input.

        A 1D raster (linescan) can be achieved by passing num_steps = 1 for two of
        the frozen axes. For example, a horizontal scan should use num_steps=(x, 1, 1)
        where x > 1 and a vertical scan should use num_steps=(1, y, 1) where y > 1.

        A 2D raster scan (area map) be achieved by passing num_steps=(x, y, 1)
        where x, y > 1

        Duplicating a coordinate in start_pos and end_pos and using a step size > 1
        in that same dimension will repeat a measurement.

        Examples
        --------
        >>> stage.raster((1e-3, 1e-3, 1e-3), (2e-3, 3e-3, 4e-3), (5, 5, 5))
        # Move to position (1mm, 1mm, 1mm) on the stage, then move along the
        # x-axis to (2mm, 1mm, 1mm) in 5 steps such that x values stop at
        # 1.00mm, 1.25mm, 1.50mm, 1.75mm, 2.00mm. Then move one unit in the
        # y-direction, where one unit is (3mm - 1 mm) / (5 - 1 steps). The
        # non-intuitive (n - 1) steps can be reasoned about by realizing that
        # the first step is applied to the initial coordinate, so there are
        # (n - 1) stops before arriving at the stopping coordinate. The
        # movement pattern continues such that the x-coordinates are swept
        # again through their stopping points, incrementing the y-step after
        # each sweep until the stopping y-coordinate is reached. The z-coordinate
        # is then incremented one step and the area sweep is carried out for
        # each step along z. The total number of stops is 125 (5 x 5 x 5) and
        # the final position after the raster is (2mm, 3mm, 4mm).

        >>> stage.raster((1e-3, 1e-3, 1e-3), (2e-3, 3e-3, 4e-3), (5, 1, 1))
        # A line scan along the x-axis, starting at (1mm, 1mm, 1mm) and stopping
        # (5 - 1) more times until the position reaches (2mm, 1mm, 1mm) (i.e. the
        # 5th stop). Both y-steps and z-steps are 1, so no further movement occurs,
        # meaning that the ending coordinates for y and z are irrelvant.

        >>> stage.raster((1e-3, 1e-3, 1e-3), (2e-3, 3e-3, 4e-3), (1, 3, 1), reset=true)
        # A line scan along the y-axis. The stage moves to (1mm, 1mm, 1mm) and
        # stops 0 more times along the x-axis because only one step is specified
        # for x. The stage moves along the y stopping points until the y-coordinate
        # is reached. Since the x-steps and z-steps are 1, the stopping coordinates
        # for x and z are ignored. The stage will have stopped at points
        # (1mm, 1mm, 1mm), (1mm, 2mm, 1mm), (1mm, 3mm, 1mm). The stage then moves
        # back to the starting position (1mm, 1mm, 1mm) because reset=true.

        >>> stage.raster((1e-3, 1e-3, 1e-3), (2e-3, 1e-3, 1e-3), (1, 3, 1))
        # A "line scan" along the y-axis (because x-steps=1 and z-steps=1), but because
        # the y-coordinate is the same in start_pos and stop_pos, this effectively
        # takes 3 measurements of the same coordinate: (1mm, 1mm, 1mm). Again, the
        # x and z coordinates in the stop_pos are ignored.
        """

        x_start, y_start, z_start = start_pos
        x_stop, y_stop, z_stop = stop_pos
        x_size, y_size, z_size = num_steps
        log.info(f'Starting raster scan from {start_pos} to {stop_pos} '
                 f'with steps {num_steps}')
        coordinates = []  # Empty list to store location of each coordinate
        data_values = []  # Empty list to store results of each coordinate
        for z in np.linspace(z_start, z_stop, z_size):
            for y in np.linspace(y_start, y_stop, y_size):
                for x in np.linspace(x_start, x_stop, x_size):
                    self.move_to(x, y, z)
                    coordinates.append((x, y, z))
                    self.wait_move()
                    log.debug(f'{self.get_position()}')
                    # Insert event logic here
                    if callback:
                        c_return = callback()
                        data_values.append(c_return)
        log.debug('Raster scan complete')
        # Store values as formatted array of shape (z, y, x, value)
        if len(data_values) == 0:
            data_values = None
        else:
            data_values = np.asarray(data_values).reshape(*num_steps[::-1], -1)
        # Store coordinates as formatted array of shape (z, y, x, 3D_coordinate)
        self.coordinates = np.asarray(coordinates).reshape(*num_steps[::-1], -1)
        if reset:
            log.info('Resetting position')
            self.move_to(x_start, y_start, z_start)
            self.wait_move()
            log.info(f'{self.get_position()}')
        return data_values
