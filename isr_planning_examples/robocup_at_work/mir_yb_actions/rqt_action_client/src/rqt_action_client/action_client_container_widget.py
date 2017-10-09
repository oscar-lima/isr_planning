import os
import time

import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, qWarning, Signal
from python_qt_binding.QtGui import QWidget, QGridLayout, QPushButton

from action_client_widget import ActionClientWidget


class ActionClientContainerWidget(QWidget):

    def __init__(self, context, actions):
        super(ActionClientContainerWidget, self).__init__()

        grid = QGridLayout()
        grid.setSpacing(1)

        MAX_COLUMNS = 2

        self.setObjectName('ActionClientContainerWidget')

        self.clear_button = QPushButton("Clear all")
        self.clear_button.clicked[bool].connect(self._handle_clear_clicked)
        grid.addWidget(self.clear_button, 0, 0)

        self.widgets = []

        row = 1
        column = 0
        for k in sorted(actions.keys()):
            action_name = k
            widget = ActionClientWidget(context, action_name, actions[k])
            grid.addWidget(widget, row, column)
            self.widgets.append(widget)
            column += 1
            if column >= MAX_COLUMNS:
                row += 1
                column = 0

        self.setLayout(grid)

    def _handle_clear_clicked(self, checked):
        for w in self.widgets:
            w.clear()
