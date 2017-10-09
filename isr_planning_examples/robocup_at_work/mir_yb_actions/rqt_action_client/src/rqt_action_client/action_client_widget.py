import os
import time
import re
import ast

import rospy
import rospkg
import actionlib
import roslib
import roslib.packages
import roslib.message

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, qWarning, Signal
from python_qt_binding.QtGui import QWidget, QGraphicsView, QGridLayout, QPushButton, QButtonGroup, QVBoxLayout, QFrame, QLineEdit

class ActionClientGraphicsView(QGraphicsView):
    def __init__(self, parent=None):
        super(ActionClientGraphicsView, self).__init__()

class ActionClientWidget(QWidget):

    def __init__(self, context, action_name, action_config):
        super(ActionClientWidget, self).__init__()

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_action_client'), 'src', 'rqt_action_client', 'resources', 'ActionClientWidget.ui')
        loadUi(ui_file, self, {'ActionClientGraphicsView': ActionClientGraphicsView})

        self.setObjectName('ActionClientWidget')

        action_config['action_name'] = action_name

        self.set_action_messages(action_config)
        self.set_ui_labels(action_config)

        if not self.client.wait_for_server(rospy.Duration(0.5)):
            rospy.logerr("Action Server %s not up", action_config['action_server'])
            self.button_send_goal.setEnabled(False)
        self.button_clear.clicked[bool].connect(self._handle_clear_clicked)
        self.button_send_goal.clicked[bool].connect(self._handle_send_goal_clicked)
        self.button_cancel_goal.clicked[bool].connect(self._handle_cancel_goal_clicked)
        self.timer = rospy.Timer(rospy.Duration(2.0), self.timer_callback)

    def timer_callback(self, event):
        if self.client.wait_for_server(rospy.Duration(0.1)):
            self.button_send_goal.setEnabled(True)
            self.button_cancel_goal.setEnabled(True)
        else:
            self.button_send_goal.setEnabled(False)
            self.button_cancel_goal.setEnabled(False)

    def create_client_and_publish_goal(self):
        slots = self.goal_class.__slots__
        goal_txt = [self.text_goal_1, self.text_goal_2, self.text_goal_3]
        #for i, s in enumerate(slots):
        for i, s in enumerate(slots):
            if type(s) != str:
                continue
            if i >= 3:
                break
            self.goal_class.__setattr__(self.goal, s, str(goal_txt[i].text()))
        self.client.send_goal(self.goal, done_cb = self.done_cb, feedback_cb = self.feedback_cb)

    def done_cb(self, terminal_state, result):
        self.text_result.setText(str(result.success))

    def feedback_cb(self, feedback):
        if feedback.current_state:
            self.text_feedback_1.setText(str(feedback.current_state))
        if feedback.text:
            self.text_feedback_2.setText(str(feedback.text))

    def _handle_send_goal_clicked(self, checked):
        self.create_client_and_publish_goal()

    def _handle_cancel_goal_clicked(self, checked):
        self.client.cancel_all_goals()

    def _handle_clear_clicked(self, checked):
        self.clear()

    def set_action_messages(self, action_config):
        exec('import ' + action_config['package'] + '.msg')
        self.action_class = roslib.message.get_message_class(action_config['package'] + '/' + action_config['action_name'] + 'Action')
        self.goal_class = roslib.message.get_message_class(action_config['package'] + '/' + action_config['action_name'] + 'Goal')
        exec('self.goal = ' + action_config['package'] + '.msg.' + action_config['action_name'] + 'Goal()')
        self.client = actionlib.SimpleActionClient(action_config['action_server'], self.action_class)

    def set_ui_labels(self, action_config):
        self.label_action_name.setText(action_config['action_name'])
        self.label_action_name.setToolTip(action_config['action_server'])
        goal_labels = [self.label_goal_1, self.label_goal_2, self.label_goal_3]
        goal_txt = [self.text_goal_1, self.text_goal_2, self.text_goal_3]
        for i, s in enumerate(self.goal_class.__slots__):
            if type(s) != str:
                continue
            if i >= 3:
                break
            goal_labels[i].setText(s)
        diff = len(goal_labels) - len(self.goal_class.__slots__)
        length = len(goal_labels)
        for i in xrange(diff):
            goal_labels[length - i - 1].setText("")
            goal_txt[length - i - 1].setEnabled(False)

        self.label_feedback_1.setText("current_state")
        self.label_feedback_2.setText("text")
        self.label_result.setText("success")

    def clear(self):
        self.text_feedback_1.setText("")
        self.text_feedback_2.setText("")
        self.text_result.setText("")
