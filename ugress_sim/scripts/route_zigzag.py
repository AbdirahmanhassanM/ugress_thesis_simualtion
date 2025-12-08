#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time
import csv

class ZigzagRoute:
    def __init__(self):
        rospy.init_node('zigzag_route')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f'/home/abdirahmanhassan/route_zigzag_{timestamp}.csv'
        self.csv_file = open(self.csv_filename, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Segment', 'Action', 'Value', 'Duration', 'Status'])
        rospy.loginfo(f"Zigzag Route - Logging to: {self.csv_filename}")
        
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
        rospy.loginfo("ROUTE: ZIGZAG PATTERN (45° alternating turns)")
        rospy.loginfo("="*60)
        rospy.sleep(3)
        start_time = time.time()
        
        # Zigzag: Alternating 45° turns
        zigzag = [
            (2.0, 45),    # Forward, turn right 45°
            (2.0, -90),   # Forward, turn left 90°
            (2.0, 90),    # Forward, turn right 90°
            (2.0, -90),   # Forward, turn left 90°
            (2.0, 90),    # Forward, turn right 90°
            (2.0, -45),   # Forward, straighten out
        ]
        
        for i, (distance, angle) in enumerate(zigzag, 1):
            rospy.loginfo(f"\n--- Segment {i}/{len(zigzag)} ---")
            
            # Drive
            duration = self.move_forward(distance)
            self.csv_writer.writerow([i, "Drive", f"{distance}m", f"{duration:.2f}s", "Success"])
            self.csv_file.flush()
            
            # Turn
            duration = self.turn(angle)
            self.csv_writer.writerow([i, "Turn", f"{angle}°", f"{duration:.2f}s", "Success"])
            self.csv_file.flush()
        
        total_time = time.time() - start_time
        rospy.loginfo("="*60)
        rospy.loginfo(f"ZIGZAG COMPLETE! Time: {total_time:.1f}s")
        rospy.loginfo("="*60)
        self.csv_file.close()

if __name__ == '__main__':
    try:
        route = ZigzagRoute()
        route.execute()
    except rospy.ROSInterruptException:
        pass
