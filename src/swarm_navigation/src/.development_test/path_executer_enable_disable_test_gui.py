#!/usr/bin/env python3

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton
import PyQt5.QtCore as qt_core
from PyQt5.QtGui import QFont

import rospy

from std_srvs.srv import Trigger, TriggerRequest
from std_srvs.srv import SetBool, SetBoolRequest

class ServiceCallButton(QPushButton):  # Now this directly inherits from QPushButton
    def __init__(self, service_address, text):
        super().__init__(text)  # Set the button text here
        
        # self.setFont(QFont('Ubuntu',13))
        self.setFont(QFont('Arial',18))
        self.service_address = service_address
        self.pressed.connect(self.button_clicked)
        
    def button_clicked(self):
        rospy.loginfo(f"Test GUI: {self.text()} Button pressed")

        rospy.wait_for_service(self.service_address, timeout=1.)
        try:
            service = rospy.ServiceProxy(self.service_address, Trigger)
            req = TriggerRequest()
            result = service(req)
        except rospy.ServiceException:
            rospy.logerr("Service call failed")

class ToggleServiceButton(QPushButton): 
    def __init__(self, service_address, text):
        super().__init__(text)
        self.setFont(QFont('Arial',18))
        self.service_address = service_address
        self.setCheckable(True)
        self.setChecked(False)
        self.clicked.connect(self.button_toggled)  # Use clicked signal

    def button_toggled(self):
        rospy.loginfo(f"Test GUI: {self.text()} Button pressed")
        desired_state = self.isChecked()
        # print("desired_state for adjusting enabled: ", desired_state)

        try:
            rospy.wait_for_service(self.service_address, timeout=1.)
            service = rospy.ServiceProxy(self.service_address, SetBool)
            req = SetBoolRequest()
            req.data = desired_state

            result = service(req)
            if result:
                if desired_state:
                    # Make the other buttons clickable
                    pass  # Implement logic here
                else:
                    # Make the other buttons unclickable
                    pass  # Implement logic here
            else:
                self.toggle_button_state()  # Toggle the state back
                return  # Don't alter the button state

        except rospy.ServiceException:
            rospy.logerr("Service call failed")
            self.toggle_button_state()  # Toggle the state back
            return  # Don't alter the button state
        except rospy.ROSException:
            rospy.logerr(f"Failed to contact service: {self.service_address}")
            self.toggle_button_state()  # Toggle the state back
            return  # Don't alter the button state

    def toggle_button_state(self):
        # Simply toggles the current state of the button without triggering the signals
        self.blockSignals(True)
        self.setChecked(not self.isChecked())
        self.blockSignals(False)


class SimpleGUI(QWidget):
    def __init__(self):
        super(SimpleGUI, self).__init__()
        self.shutdown_timer = qt_core.QTimer()

        self.layout = QVBoxLayout()

        self.disable_button = ServiceCallButton("/disable_path_execution", "Disable Execution")
        self.enable_button = ServiceCallButton("/enable_path_execution", "Enable Execution")
        self.cancel_button = ServiceCallButton("/cancel_path_execution", "Cancel Execution")
        self.toggle_adjust_path_button = ToggleServiceButton("/toggle_adjust_path", "Adjust Path")

        self.layout.addWidget(self.disable_button)
        self.layout.addWidget(self.enable_button)
        self.layout.addWidget(self.cancel_button)
        self.layout.addWidget(self.toggle_adjust_path_button)

        self.setLayout(self.layout)

        self.shutdown_timer.timeout.connect(self.check_shutdown)
        self.shutdown_timer.start(1000)  # Timer triggers every 1000 ms (1 second)

    def check_shutdown(self):
        if rospy.is_shutdown():
            QApplication.quit()

if __name__ == '__main__':
    rospy.init_node('path_executer_enable_disable_test_gui', anonymous=True)

    app = QApplication(sys.argv)
    window = SimpleGUI()
    window.show()
    sys.exit(app.exec_())
