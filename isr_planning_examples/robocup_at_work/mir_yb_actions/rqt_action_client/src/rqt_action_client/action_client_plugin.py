import os
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from action_client_container_widget import ActionClientContainerWidget

import yaml

class ActionClientPlugin(Plugin):

    def __init__(self, context):
        super(ActionClientPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ActionClientPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")

        parser.add_argument("-c", "--config", action="store_true",
                      dest="config_file",
                      help="path to config yaml file")

        args, unknowns = parser.parse_known_args(context.argv())

        yaml_file = unknowns[0]
        print "Loading file: ", yaml_file

        stream = open(yaml_file, "r")
        self.actions = yaml.load(stream)

        # Create QWidget
        self._widget = ActionClientContainerWidget(context, self.actions)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'resources', 'ActionClientPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('ActionClientPluginUi')
