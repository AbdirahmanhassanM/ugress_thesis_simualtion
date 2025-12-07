#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import csv
import time
from tf.transformations import euler_from_quaternion

class PathFollower:
    def __init__(self):
        rospy.init_node('path_follower', anonymous=True)
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.rate = rospy.Rate(10)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.odom_received = False
        
        self.waypoints = [
            (3.0, 0.0),
            (3.0, 0.5),
            (0.0, 0.5),
            (0.0, 1.0),
            (3.0, 1.0),
            (3.0, 1.5),
            (0.0, 1.5),
        ]
        
        self.start_time = None
        self.csv_file = None
        self.csv_writer = None
        
        self.init_logging()
        
    def init_logging(self):
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f'/home/abdirahmanhassan/path_data_{timestamp}.csv'
        self.csv_file = open(filename, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'Waypoint_Number', 
            'Target_X', 
            'Target_Y', 
            'Actual_X', 
            'Actual_Y',
            'Time_Elapsed',
            'Distance_Error',
            'Status'
        ])
        rospy.loginfo(f"Data logging to: {filename}")
        
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_theta = yaw
        
        self.odom_received = True
        
    def wait_for_odom(self):
        rospy.loginfo("Waiting for odometry data...")
        while not self.odom_received and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("Odometry data received!")
        
    def move_to_waypoint(self, waypoint_num, target_x, target_y):
        rospy.loginfo(f"Waypoint {waypoint_num}: Moving to ({target_x:.2f}, {target_y:.2f})")
        
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        rospy.loginfo(f"  Distance: {distance:.2f}m, Target angle: {math.degrees(target_angle):.1f} degrees")
        
        self.rotate_to_angle(target_angle)
        self.move_forward(distance)
        
        final_dx = target_x - self.current_x
        final_dy = target_y - self.current_y
        position_error = math.sqrt(final_dx**2 + final_dy**2)
        
        elapsed = time.time() - self.start_time
        self.csv_writer.writerow([
            waypoint_num,
            target_x,
            target_y,
            self.current_x,
            self.current_y,
            elapsed,
            position_error,
            'Success'
        ])
        self.csv_file.flush()
        
        rospy.loginfo(f"  Reached! Error: {position_error:.3f}m, Time: {elapsed:.1f}s")
        
    def rotate_to_angle(self, target_angle):
        twist = Twist()
        
        angle_diff = target_angle - self.current_theta
        
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        angular_speed = 0.5
        tolerance = 0.05
        
        while abs(angle_diff) > tolerance and not rospy.is_shutdown():
            if angle_diff > 0:
                twist.angular.z = angular_speed
            else:
                twist.angular.z = -angular_speed
            
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()
            
            angle_diff = target_angle - self.current_theta
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
        
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(0.3)
        
    def move_forward(self, distance):
        twist = Twist()
        
        start_x = self.current_x
        start_y = self.current_y
        
        linear_speed = 0.3
        tolerance = 0.05
        
        while not rospy.is_shutdown():
            dx = self.current_x - start_x
            dy = self.current_y - start_y
            traveled = math.sqrt(dx**2 + dy**2)
            
            remaining = distance - traveled
            
            if remaining < tolerance:
                break
            
            if remaining < 0.5:
                twist.linear.x = linear_speed * 0.5
            else:
                twist.linear.x = linear_speed
            
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()
        
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(0.5)
        
    def follow_path(self):
        rospy.loginfo("="*50)
        rospy.loginfo("PATH FOLLOWING STARTED")
        rospy.loginfo("="*50)
        
        self.wait_for_odom()
        rospy.sleep(2)
        
        self.start_time = time.time()
        
        for i, waypoint in enumerate(self.waypoints, 1):
            if rospy.is_shutdown():
                break
            self.move_to_waypoint(i, waypoint[0], waypoint[1])
        
        total_time = time.time() - self.start_time
        rospy.loginfo("="*50)
        rospy.loginfo(f"PATH COMPLETE! Total time: {total_time:.1f}s")
        rospy.loginfo(f"Waypoints visited: {len(self.waypoints)}")
        rospy.loginfo("="*50)
        
        self.csv_file.close()
        
    def cleanup(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        if self.csv_file:
            self.csv_file.close()

if __name__ == '__main__':
    try:
        follower = PathFollower()
        follower.follow_path()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'follower' in locals():
            follower.cleanup()
