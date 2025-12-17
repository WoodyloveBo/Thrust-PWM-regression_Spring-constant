#!/usr/bin/env python3
import glob
import os
import rosbag
import rospy
import sys

def shift_timestamps(input_bag_path, output_bag_path, offset_sec):
    """
    /cf2/log1 및 /cf2/pose 토픽의 header.stamp를 offset_sec(초) 만큼 뺀 후
    새로운 bag 파일에 저장합니다.
    """
    inbag = rosbag.Bag(input_bag_path, 'r')
    outbag = rosbag.Bag(output_bag_path, 'w')
    offset = rospy.Duration(offset_sec)

    for topic, msg, t in inbag.read_messages():
        if topic in ['/cf2/log1', '/cf2/pose']:
            new_stamp = msg.header.stamp - offset
            msg.header.stamp = new_stamp
            outbag.write(topic, msg, new_stamp)
        else:
            outbag.write(topic, msg, t)

    inbag.close()
    outbag.close()
    rospy.loginfo("Processed '%s' -> '%s' (shifted by %.2f sec)",
                  input_bag_path, output_bag_path, offset_sec)

def main():
    rospy.init_node('batch_shift_timestamps', anonymous=True)
    # 시프팅할 초 단위 오프셋 설정
    offset_sec = 2.2

    # 현재 디렉토리에서 9g_T?.bag 패턴에 매칭되는 모든 파일 찾기
    bag_pattern = '14g_T?_.bag'
    bag_files = glob.glob(bag_pattern)

    if not bag_files:
        rospy.logwarn("No bag files matching pattern '%s' found.", bag_pattern)
        sys.exit(1)

    for infile in bag_files:
        # 출력 파일명: 예) 9g_T1.bag -> 9g_T1_shifted.bag
        base, ext = os.path.splitext(infile)
        outfile = f"{base}offset{ext}"
        shift_timestamps(infile, outfile, offset_sec)

if __name__ == '__main__':
    main()
