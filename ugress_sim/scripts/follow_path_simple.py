#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time
import csv

rospy.init_node('simple_path')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)

# CSV logging
timestamp = time.strftime("%Y%m%d_%H%M%S")
csv_file = open(f'/home/abdirahmanhassan/simple_path_{timestamp}.csv', 'w')
writer = csv.writer(csv_file)
writer.writerow(['Segment', 'Action', 'Duration', 'Status'])

print("Starting in 3 seconds...")
rospy.sleep(3)
start_time = time.time()

# Define movements: (linear_speed, angular_speed, duration, description)
movements = [
    (0.3, 0.0, 10.0, "Forward 3m"),
    (0.0, 0.5, 3.14, "Turn 90°"),
    (0.3, 0.0, 1.67, "Forward 0.5m"),
    (0.0, 0.5, 3.14, "Turn 90°"),
    (0.3, 0.0, 10.0, "Forward 3m"),
    (0.0, 0.5, 3.14, "Turn 90°"),
    (0.3, 0.0, 1.67, "Forward 0.5m"),
    (0.0, 0.5, 3.14, "Turn 90°"),
    (0.3, 0.0, 10.0, "Forward 3m"),
]

twist = Twist()

for i, (linear, angular, duration, desc) in enumerate(movements, 1):
    print(f"Segment {i}: {desc}")
    
    twist.linear.x = linear
    twist.angular.z = angular
    
    t_start = rospy.Time.now()
    while (rospy.Time.now() - t_start).to_sec() < duration:
        pub.publish(twist)
        rate.sleep()
    
    # Stop
    twist.linear.x = 0
    twist.angular.z = 0
    pub.publish(twist)
    rospy.sleep(0.5)
    
    elapsed = time.time() - start_time
    writer.writerow([i, desc, f"{duration:.2f}", "Success"])
    csv_file.flush()

print("PATH COMPLETE!")
csv_file.close()
