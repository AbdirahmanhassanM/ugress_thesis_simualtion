#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time
import csv

class CircleRoute:
    def __init__(self):
        rospy.init_node('circle_route')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f'/home/abdirahmanhassan/route_circle_{timestamp}.csv'
        self.csv_file = open(self.csv_filename, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Segment', 'Action', 'Value', 'Duration', 'Status'])
        rospy.loginfo(f"Circle Route - Logging to: {self.csv_filename}")
        
    def stop_robot(self, pause=1.0):
        twist = Twist()
        for _ in range(20):
            self.pub.publish(twist)
            rospy.sleep(0.05)
        rospy.sleep(pause)
            
    def move_forward(self, distance, speed=0.3):
        duration = distance / speed
        rospy.loginfo(f"  Moving forward {distance}m")
        twist = Twist()
        twist.linear.x = speed
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < duration and not rospy.is_shutdown():
            self.pub.publish(twist)
            self.rate.sleep()
        self.stop_robot()
        return duration
        
    def turn(self, angle_degrees, angular_speed=0.5):
        import math
        angle_rad = math.radians(abs(angle_degrees))
        duration = angle_rad / angular_speed
        rospy.loginfo(f"  Turning {angle_degrees}°")
        twist = Twist()
        twist.angular.z = angular_speed if angle_degrees > 0 else -angular_speed
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < duration and not rospy.is_shutdown():
            self.pub.publish(twist)
            self.rate.sleep()
        self.stop_robot()
        return duration
        
    def execute(self):
        rospy.loginfo("="*60)
        rospy.loginfo("ROUTE: CIRCLE PATTERN (12 segments, 30° turns)")
        rospy.loginfo("="*60)
        rospy.sleep(3)
        start_time = time.time()
        
        # Circle: 12 short segments with 30° turns (360°/12 = 30°)
        for i in range(12):
            rospy.loginfo(f"\n--- Segment {i+1}/12 ---")
            
            # Drive short distance
            duration = self.move_forward(0.8)
            self.csv_writer.writerow([i+1, "Drive", "0.8m", f"{duration:.2f}s", "Success"])
            self.csv_file.flush()
            
            # Turn 30°
            duration = self.turn(30)
            self.csv_writer.writerow([i+1, "Turn", "30°", f"{duration:.2f}s", "Success"])
            self.csv_file.flush()
        
        total_time = time.time() - start_time
        rospy.loginfo("="*60)
        rospy.loginfo(f"CIRCLE COMPLETE! Time: {total_time:.1f}s")
        rospy.loginfo("="*60)
        self.csv_file.close()

if __name__ == '__main__':
    try:
        route = CircleRoute()
        route.execute()
    except rospy.ROSInterruptException:
        pass
