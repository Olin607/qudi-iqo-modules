import time
import numpy as np
import logging as log


class Scanner:

    def __init__(self, x_motor, y_motor, z_motor):
        self.X_motor = x_motor
        self.Y_motor = y_motor
        self.Z_motor = z_motor

        self.start_pos = None
        self.stop_pos = None
        self.num_steps = None

    def is_moving(self):
        """Query the status of all three axes; return false only when all three motors are stopped."""
        return self.X_motor.is_moving() or self.Y_motor.is_moving() or self.Z_motor.is_moving()

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
        print('PLEASE BE AWARE THAT BACKLASH MAY OCCUR IN STAGE MOVEMENT!!!\n')
        print(f'Enter absolute starting position in {user_scale} '
              'or leave blank to use current position. Make sure you are considering'
              'Make sure to keep account of the possibility that back-lashings occurs.')
        start_query = input('(x_start, y_start, z_start): ')
        if start_query == '':
            self.start_pos = self.get_position()
        else:
            self.start_pos = [unit_conv * float(val) for val in start_query.split()]

        print(f'Enter absolute stopping position in {user_scale} '
              'or leave blank to use current position.')
        stop_query = input('(x_stop, y_stop, z_stop): ')
        if stop_query == '':
            self.stop_pos = self.get_position()
        else:
            self.stop_pos = [unit_conv * float(val) for val in stop_query.split()]

        print('Enter number of steps or leave blank for single measurement: ')
        num_query = input('(x_steps, y_steps, z_steps): ')
        if num_query == '':
            self.num_steps = [1, 1, 1]
        else:
            self.num_steps = [int(float(num)) for num in num_query.split()]

        answer = input('Continue? ([Y]/n): ')
        if answer.lower() not in ['', 'y', 'yes']:
            print('Aborting script as prompt was not answered correctly.')
            exit()
        else:
            print(f'\nThe following scan will be conducted:\n'
              f'start_pos\t=\t{self.start_pos} m\n'
              f'stop_pos\t=\t{self.stop_pos} m\n'
              f'num_steps\t=\t{self.num_steps}\n'
              f'Total measurements\t{np.product(self.num_steps)}\n')

    def raster(self, callback=None, reset=False):
        """Move the stage from start to stop in a specified number of steps.

        Parameters
        ----------
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
        >>> stage.query()
        >>> stage.raster(1e-3 1e-3 1e-3, 2e-3, 3e-3, 4e-3), (5, 5, 5))
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

        x_start, y_start, z_start = self.start_pos
        x_stop, y_stop, z_stop = self.stop_pos
        x_size, y_size, z_size = self.num_steps

        x_points = np.linspace(x_start, x_stop, x_size)
        y_points = np.linspace(y_start, y_stop, y_size)
        z_points = np.linspace(z_start, z_stop, z_size)

        log.info(f'Starting raster scan from {self.start_pos} to {self.stop_pos} '
                 f'with steps {self.num_steps}')
        coordinates = []  # Empty list to store location of each coordinate
        data_values = []  # Empty list to store results of each coordinate
        for z in z_points:
            for y in y_points:
                for x in x_points:
                    self.move_abs(x, y, z)
                    coordinates.append((x, y, z))
                    self.wait_move()
                    log.debug(f'{self.get_pos()}')
                    # Insert event logic here
                    if callback:
                        c_return = callback()
                        data_values.append(c_return)
        log.debug('Raster scan complete')
        # Store values as formatted array of shape (z, y, x, value)
        if len(data_values) == 0:
            data_values = None
        else:
            data_values = np.asarray(data_values).reshape(*self.num_steps[::-1], -1)
        # Store coordinates as formatted array of shape (z, y, x, 3D_coordinate)
        self.coordinates = np.asarray(coordinates).reshape(*self.num_steps[::-1], -1)
        if reset:
            log.info('Resetting position')
            self.move_abs(x_start, y_start, z_start)
            self.wait_move()
            log.info(f'{self.get_pos()}')
        return data_values

    def move_abs(self, x, y, z):
        """Move the motors to the specified x, y, and z coordinates in absolute terms."""
        # Assuming that the move_to method in pylablib moves the motor to an absolute position
        self.X_motor.move_to(x)
        self.Y_motor.move_to(y)
        self.Z_motor.move_to(z)
        # Wait for all motors to stop moving
        self.wait_move()

    def get_pos(self):
        """ Gets current position of the stage arms

        @param list param_list: optional, if a specific position of an axis
                                is desired, then the labels of the needed
                                axis should be passed in the param_list.
                                If nothing is passed, then from each axis the
                                position is asked.

        @return dict: with keys being the axis labels and item the current
                      position.
        """
        positions = {
            'x': self.X_motor.get_position(),
            'y': self.Y_motor.get_position(),
            'z': self.Z_motor.get_position()
        }
        return positions
