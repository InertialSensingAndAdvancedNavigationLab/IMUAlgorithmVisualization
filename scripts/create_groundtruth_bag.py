#!/usr/bin/env python

import rospy
import rosbag
import os
import csv
from geometry_msgs.msg import PoseStamped

def create_groundtruth_bag(csv_path, bag_path):
    """
    Reads a EuRoC-style groundtruth CSV file and writes it to a rosbag.
    The rosbag will contain geometry_msgs/PoseStamped messages on the 
    /ground_truth/pose topic.

    CSV format:
    timestamp [ns], p_x [m], p_y [m], p_z [m], q_w, q_x, q_y, q_z, ...
    """
    if not os.path.exists(csv_path):
        rospy.logerr("Input CSV file not found: %s", csv_path)
        return

    rospy.loginfo("Reading ground truth data from %s", csv_path)
    rospy.loginfo("Writing to rosbag at %s", bag_path)

    try:
        with rosbag.Bag(bag_path, 'w') as bag:
            with open(csv_path, 'r') as csvfile:
                reader = csv.reader(csvfile)
                # Skip header if it exists
                header = next(reader)
                if 'timestamp' not in header[0]:
                    # Reset file iterator if there was no header
                    csvfile.seek(0)

                for row in reader:
                    if not row or row[0].strip().startswith('#'):
                        continue

                    # --- Create PoseStamped Message ---
                    pose_msg = PoseStamped()

                    # Timestamp (ns to rospy.Time)
                    timestamp_ns = int(row[0])
                    pose_msg.header.stamp = rospy.Time.from_sec(timestamp_ns / 1e9)
                    pose_msg.header.frame_id = "world"

                    # Position
                    pose_msg.pose.position.x = float(row[1])
                    pose_msg.pose.position.y = float(row[2])
                    pose_msg.pose.position.z = float(row[3])

                    # Orientation (CSV: w, x, y, z -> ROS: x, y, z, w)
                    pose_msg.pose.orientation.w = float(row[4])
                    pose_msg.pose.orientation.x = float(row[5])
                    pose_msg.pose.orientation.y = float(row[6])
                    pose_msg.pose.orientation.z = float(row[7])

                    # Write to bag
                    bag.write('/ground_truth/pose', pose_msg, pose_msg.header.stamp)

    except Exception as e:
        rospy.logerr("An error occurred: %s", str(e))
        return

    rospy.loginfo("Successfully created rosbag file at %s", bag_path)

if __name__ == '__main__':
    # --- HARD-CODED PATHS ---
    # Input CSV file containing the ground truth data.
    input_csv_path = "/mnt/d/中北大学惯性传感与先进导航实验室/IMU-GNSS/mahony_madwick/data/MH_04_difficult/mav0/state_groundtruth_estimate0/data.csv"
    
    # Desired output path for the new rosbag file.
    output_bag_path = "/mnt/d/Ubuntu20ROS1/Euroc_ws/MH_04_difficult_true.bag"
    
    rospy.init_node('create_groundtruth_bag', anonymous=True)
    create_groundtruth_bag(input_csv_path, output_bag_path)
