#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time
import csv

class PathFollower:
    def __init__(self):
        rospy.init_node('path_follower')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        
        # Initialize CSV logging
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f'/home/abdirahmanhassan/simulation_run_{timestamp}.csv'
        self.csv_file = open(self.csv_filename, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Cycle', 'Action', 'Distance_or_Angle', 'Duration', 'Status'])
        
        rospy.loginfo(f"Logging to: {self.csv_filename}")
        self.start_time = None
        
    def stop_robot(self, pause_duration=1.0):
        """Stop robot completely and pause"""
        rospy.loginfo(f"  STOPPING - Pausing for {pause_duration}s")
        twist = Twist()
        for _ in range(20):
            self.pub.publish(twist)
            rospy.sleep(0.05)
        rospy.sleep(pause_duration)
            
    def move_forward(self, distance, speed=0.3):
        """Move forward for specified distance"""
        duration = distance / speed
        rospy.loginfo(f"  DRIVING forward {distance}m at {speed}m/s")
        
        twist = Twist()
        twist.linear.x = speed
        
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < duration and not rospy.is_shutdown():
            self.pub.publish(twist)
            self.rate.sleep()
        
        self.stop_robot(pause_duration=1.0)
        return duration
        
    def turn(self, angle_degrees, angular_speed=0.5):
        """Turn by specified angle"""
        import math
        angle_rad = math.radians(abs(angle_degrees))
        duration = angle_rad / angular_speed
        
        rospy.loginfo(f"  TURNING {angle_degrees}°")
        
        twist = Twist()
        if angle_degrees > 0:
            twist.angular.z = angular_speed
        else:
            twist.angular.z = -angular_speed
        
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < duration and not rospy.is_shutdown():
            self.pub.publish(twist)
            self.rate.sleep()
        
        self.stop_robot(pause_duration=1.0)
        return duration
        
    def follow_path(self):
        """Execute 9 drive-stop-turn-stop cycles"""
        rospy.loginfo("="*70)
        rospy.loginfo("STARTING 9-CYCLE PATH EXECUTION")
        rospy.loginfo("Pattern: DRIVE → STOP → TURN → STOP (repeat 9 times)")
        rospy.loginfo("="*70)
        
        rospy.sleep(3)
        self.start_time = time.time()
        
        # Define 9 cycles
        cycles = [
            {"drive": 3.0, "turn": 90},   # Cycle 1
            {"drive": 2.5, "turn": 90},   # Cycle 2
            {"drive": 3.0, "turn": 90},   # Cycle 3
            {"drive": 2.5, "turn": 90},   # Cycle 4
            {"drive": 3.0, "turn": 90},   # Cycle 5
            {"drive": 2.5, "turn": 90},   # Cycle 6
            {"drive": 3.0, "turn": 90},   # Cycle 7
            {"drive": 2.5, "turn": 90},   # Cycle 8
            {"drive": 3.0, "turn": 0},    # Cycle 9 (final, no turn)
        ]
        
        for i, cycle in enumerate(cycles, 1):
            if rospy.is_shutdown():
                break
            
            rospy.loginfo("")
            rospy.loginfo("="*70)
            rospy.loginfo(f"CYCLE {i}/9")
            rospy.loginfo("="*70)
            
            # STEP 1: DRIVE
            rospy.loginfo(f"Step 1: DRIVE {cycle['drive']}m")
            drive_duration = self.move_forward(cycle['drive'])
            self.csv_writer.writerow([i, "Drive", f"{cycle['drive']}m", f"{drive_duration:.2f}s", "Success"])
            self.csv_file.flush()
            
            # STEP 2: TURN (if not last cycle)
            if cycle['turn'] > 0:
                rospy.loginfo(f"Step 2: TURN {cycle['turn']}°")
                turn_duration = self.turn(cycle['turn'])
                self.csv_writer.writerow([i, "Turn", f"{cycle['turn']}°", f"{turn_duration:.2f}s", "Success"])
                self.csv_file.flush()
            else:
                rospy.loginfo("Step 2: Final cycle - No turn")
            
            rospy.loginfo(f"✓ Cycle {i} complete!")
        
        # Final summary
        total_time = time.time() - self.start_time
        rospy.loginfo("")
        rospy.loginfo("="*70)
        rospy.loginfo("ALL 9 CYCLES COMPLETE!")
        rospy.loginfo(f"Total cycles: 9")
        rospy.loginfo(f"Total time: {total_time:.1f} seconds")
        rospy.loginfo(f"Average time per cycle: {total_time/9:.1f} seconds")
        rospy.loginfo(f"Data saved to: {self.csv_filename}")
        rospy.loginfo("="*70)
        
        self.csv_file.close()
        
    def cleanup(self):
        """Stop robot and cleanup"""
        self.stop_robot(pause_duration=0)
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
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
