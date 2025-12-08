#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time
import csv

class SquareRoute:
    def __init__(self):
        rospy.init_node('square_route')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f'/home/abdirahmanhassan/route_square_{timestamp}.csv'
        self.csv_file = open(self.csv_filename, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Segment', 'Action', 'Value', 'Duration', 'Status'])
        rospy.loginfo(f"Square Route - Logging to: {self.csv_filename}")
        
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
        rospy.loginfo("ROUTE: SQUARE PATTERN (4 sides, 90° turns)")
        rospy.loginfo("="*60)
        rospy.sleep(3)
        start_time = time.time()
        
        # Square: 4 sides of 3 meters each
        for i in range(4):
            rospy.loginfo(f"\n--- Side {i+1}/4 ---")
            
            # Drive forward
            duration = self.move_forward(3.0)
            self.csv_writer.writerow([i+1, "Drive", "3.0m", f"{duration:.2f}s", "Success"])
            self.csv_file.flush()
            
            # Turn 90° (unless last side)
            if i < 3:
                duration = self.turn(90)
                self.csv_writer.writerow([i+1, "Turn", "90°", f"{duration:.2f}s", "Success"])
                self.csv_file.flush()
        
        total_time = time.time() - start_time
        rospy.loginfo("="*60)
        rospy.loginfo(f"SQUARE COMPLETE! Time: {total_time:.1f}s")
        rospy.loginfo("="*60)
        self.csv_file.close()

if __name__ == '__main__':
    try:
        route = SquareRoute()
        route.execute()
    except rospy.ROSInterruptException:
        pass
