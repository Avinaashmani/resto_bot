#!/usr/bin/env python3

import time
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool, String, Int16MultiArray
from std_srvs.srv import Trigger, TriggerResponse

class RestoBotController:

    def __init__(self):
        rospy.init_node('resto_bot')
        rospy.loginfo_once("Resto Bot Controller Has Begun")

        self.locations = {
            "home": (-2.0, -1.5, 0.025, 1.0),
            "kitchen": (-1.3, -0.5, -0.03, 1.0),
            "table_1": (-0.6, 0.6, 0.0038, 1.0),
            "table_2": (0.5, 0.6, -0.06, 1.0),
            "table_3": (0.6, 2.0, 1.0, 0.003)
        }

        self.order_list = []
        self.current_order = None
        self.kitchen_confirmation = False
        self.table_confirmation = False

        self.diagnostics_pub = rospy.Publisher('/resto_bot/diagnostics', String, queue_size=10)
        rospy.Subscriber('/resto_bot/kitchen_confirmation', Bool, self.kitchen_cb)
        rospy.Subscriber('/resto_bot/table_confirmation', Bool, self.table_cb)
        rospy.Subscriber('/resto_bot/table_order', Int16MultiArray, self.table_orders)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.service = rospy.Service('/resto_bot/process_orders', Trigger, self.handle_process_orders)

    def move_to(self, location_name):
        location = self.locations[location_name]
        rospy.loginfo(f"Moving to {location_name} at {location}")
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = location[0]
        goal.target_pose.pose.position.y = location[1]
        goal.target_pose.pose.orientation.z = location[2]
        goal.target_pose.pose.orientation.w = location[3]

        self.client.send_goal(goal)
        self.client.wait_for_result()

        rospy.loginfo(f"Reached {location_name}")

    def wait_for_confirmation(self, timeout, confirmation_callback):
        rospy.loginfo(f"Waiting for confirmation with timeout {timeout} seconds")
        start_time = time.time()
        while time.time() - start_time < timeout:
            if confirmation_callback():
                rospy.loginfo("Confirmation received")
                return True
            rospy.sleep(0.1)
        rospy.loginfo("Timeout waiting for confirmation")
        return False

    def kitchen_cb(self, msg):
        self.kitchen_confirmation = msg.data

    def table_orders(self, msg):
        self.order_list = msg.data

    def table_cb(self, msg):
        self.table_confirmation = msg.data

    def handle_process_orders(self, req):
        # Reset confirmation states
        self.kitchen_confirmation = False
        self.table_confirmation = False
        
        rospy.loginfo(f"Received service call to process orders: {self.order_list}")
        self.process_orders()
        return TriggerResponse(success=True, message="Orders processed")

    def process_orders(self):
 
        for order in self.order_list:
            table_name = f"table_{order}"
            self.current_order = table_name

            self.move_to(table_name)
            self.wait_for_confirmation(4, lambda: self.table_confirmation)

        self.move_to("kitchen")
        self.wait_for_confirmation(4, lambda: self.kitchen_confirmation)

        self.move_to("home")

if __name__ == '__main__':
    try:
        RestoBotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
