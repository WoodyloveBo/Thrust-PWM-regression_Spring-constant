#!/usr/bin/env python
import glob
import os
import rosbag
from sensor_msgs.msg import PointCloud


def extract_point4_preserve(input_bag_path, output_bag_path):
    """
    모든 토픽을 그대로 복사하면서,
    '/natnet_ros/pointcloud'에서는 points[4]만 추출하여
    '/point_index4' 토픽으로 저장합니다.
    """
    inbag = rosbag.Bag(input_bag_path, 'r')
    outbag = rosbag.Bag(output_bag_path, 'w')
    
    for topic, msg, t in inbag.read_messages():
        if topic == '/natnet_ros/pointcloud':
            if len(msg.points) > 4:
                new_msg = PointCloud()
                new_msg.header = msg.header
                new_msg.points = [msg.points[4]]
                outbag.write('/point_index4', new_msg, t)
        else:
            outbag.write(topic, msg, t)
    
    inbag.close()
    outbag.close()


if __name__ == '__main__':
    # 현재 디렉토리의 9g_T?.bag 파일 모두 처리
    pattern = '10g_T?.bag'
    bags = glob.glob(pattern)
    if not bags:
        print(f"No bag files match '{pattern}'")
        exit(1)

    for infile in bags:
        base = os.path.splitext(infile)[0]
        outfile = f"{base}_.bag"
        print(f"Processing {infile} -> {outfile}")
        extract_point4_preserve(infile, outfile)
    print("Batch extraction complete.")
