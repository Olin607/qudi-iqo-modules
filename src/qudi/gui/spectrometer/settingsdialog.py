# -*- coding: utf-8 -*-
"""
This file contains a settings dialog for the qudi main GUI.

Copyright (c) 2021, the qudi developers. See the AUTHORS.md file at the top-level directory of this
distribution and on <https://github.com/Ulm-IQO/qudi-core/>

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

from PySide2 import QtCore, QtWidgets
from qudi.util.widgets.scientific_spinbox import ScienDSpinBox


class SettingsDialog(QtWidgets.QDialog):
    """
    Custom QDialog widget for configuration of the spectrometer
    """
    def __init__(self, parent=None, **kwargs):
        super().__init__(parent, **kwargs)
        self.setWindowTitle('Spectrometer settings')

        # Create main layout
        # Add widgets to layout and set as main layout
        layout = QtWidgets.QGridLayout()
        layout.setRowStretch(1, 1)
        self.setLayout(layout)

        # Create widgets and add them to the layout
        self.exposure_time_spinbox = ScienDSpinBox()
        self.exposure_time_spinbox.setMinimumWidth(150)
        exposure_time_label = QtWidgets.QLabel('Exposure Time:')
        exposure_time_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        layout.addWidget(exposure_time_label, 0, 0)
        layout.addWidget(self.exposure_time_spinbox, 0, 1)

        buttonbox = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok
                                               | QtWidgets.QDialogButtonBox.Cancel
                                               | QtWidgets.QDialogButtonBox.Apply)
        buttonbox.setOrientation(QtCore.Qt.Horizontal)
        layout.addWidget(buttonbox, 2, 0, 1, 2)

        # Add internal signals
        buttonbox.accepted.connect(self.accept)
        buttonbox.rejected.connect(self.reject)
        buttonbox.button(buttonbox.Apply).clicked.connect(self.accepted)
