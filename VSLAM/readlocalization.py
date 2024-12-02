#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class SubPoseTopic:
    def __init__(self):
        self.pose_data = PoseStamped()  # Initialise ROS msg object of type PoseStamped
        self.pose_data_cov = PoseWithCovarianceStamped()  # Initialise ROS msg object of type PoseWithCovarianceStamped
        self.pose_subscriber_zed_node()  # Subscribe to /zed/zed_node/pose
        self.pose_subscriber_localisation_pose() # Subscribe to /zed/localization_pose

    def pose_subscriber_zed_node(self):
        sub = rospy.Subscriber('/zed/zed_node/pose', PoseStamped, self.pose_callback_zed_node)
        print("subscribed to /zed/localization_pose topic")

    def pose_callback_zed_node(self, pose_data):
        self.pose_data = pose_data

    def pose_subscriber_localisation_pose(self):
        sub = rospy.Subscriber('/rtabmap/localization_pose', PoseWithCovarianceStamped, self.pose_callback_localisation_pose)
        print("subscribed to /zed/localization_pose topic")

    def pose_callback_localisation_pose(self, pose_data_cov):
        self.pose_data_cov = pose_data_cov
        
    def print_localisation_values(self):
        print(f"x = {self.pose_data.pose.position.x}, y = {self.pose_data.pose.position.y}")

    def print_localisation_values_cov(self):
        print(f"x = {self.pose_data_cov.pose.pose.position.x}, y = {self.pose_data_cov.pose.pose.position.y}")
        
    def fetch_localisation_values_from_zed_node(self):
        localised_x = self.pose_data.pose.position.x
        localised_y = self.pose_data.pose.position.y
        orientation = self.pose_data.pose.orientation
        return localised_x, localised_y, orientation
    
    def fetch_localisation_values_from_loc_pose_cov(self):
        localised_x_cov = self.pose_data_cov.pose.pose.position.x
        localised_y_cov = self.pose_data_cov.pose.pose.position.y
        orientation_cov = self.pose_data_cov.pose.pose.orientation
        return localised_x_cov, localised_y_cov, orientation_cov
    
if __name__ == '__main__':
    rospy.init_node('test_pose_subscriber_node')
    obj = SubPoseTopic()
    while not rospy.is_shutdown():
        # obj.print_localisation_values()
        obj.print_localisation_values_cov()

