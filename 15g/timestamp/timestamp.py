#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
하드코딩된 입력 bag들(10g_1_offset.bag, 10g_2_offset.bag, 10g_3_offset.bag)에 대해
아래 3개 토픽만 추려서 각 파일별로 가장 이른 시각을 0.0초로 맞춘 뒤
*_timestamp.bag으로 저장한다.

대상 토픽:
  - /cf2/log1
  - /natnet/cf2/pose
  - /point/index4/points
"""

import os
import sys
import rosbag
import rospy

# ---- 하드코딩된 입력 목록 ----
INPUT_BAGS = [
    "15g_T1_offset.bag",
    "15g_T2_offset.bag",
    "15g_T3_offset.bag",
]

# ---- 타임스탬프를 0으로 정렬할 대상 토픽 ----
TOPICS = [
    "/cf2/log1",
    "/natnet_ros/cf2/pose",
    "/point_index4",
]


def out_name(path):
    base, ext = os.path.splitext(path)
    if base.endswith("_offset"):
        base = base[:-len("_offset")]
    return base + "_timestamp" + ext


def get_first_time(bag_path, topics):
    """지정 토픽에서 가장 이른 bag 타임스탬프(rosbag record time)를 찾는다."""
    t0 = None
    cnt = 0
    with rosbag.Bag(bag_path, "r") as bag:
        for _, _, t in bag.read_messages(topics=topics):
            cnt += 1
            if t0 is None or t < t0:
                t0 = t
    if t0 is None:
        raise RuntimeError(f"[{bag_path}] 지정 토픽에 메시지가 없습니다: {topics}")
    return t0, cnt


def shift_header_stamp_if_any(msg, new_stamp):
    """std_msgs/Header가 있으면 header.stamp를 new_stamp로 맞춰준다."""
    # 일부 메시지는 header가 없거나 stamp 타입이 Time이 아닐 수 있음
    try:
        if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
            # rospy.Time로 강제
            if isinstance(new_stamp, rospy.Time):
                msg.header.stamp = new_stamp
            else:
                msg.header.stamp = rospy.Time.from_sec(float(new_stamp.to_sec()))
    except Exception:
        # header가 배열 안쪽에 있거나 타입이 다른 경우는 조용히 패스
        pass
    return msg


def process_one(bag_path):
    if not os.path.isfile(bag_path):
        print(f"[WARN] 파일 없음: {bag_path}")
        return

    out_path = out_name(bag_path)
    print(f"\n=== 처리 시작: {bag_path} -> {out_path}")
    t0, n = get_first_time(bag_path, TOPICS)
    print(f"  - 기준 시각 t0 = {t0.to_sec():.6f}s (메시지 {n}개에서 최소값)")

    wrote = 0
    with rosbag.Bag(bag_path, "r") as bag, rosbag.Bag(out_path, "w") as out:
        for topic, msg, t in bag.read_messages(topics=TOPICS):
            dt = (t - t0).to_sec()
            if dt < 0:
                # 이론상 없어야 하지만(최솟값보다 이른 시간), 안전망
                dt = 0.0
            new_t = rospy.Time.from_sec(dt)

            # header.stamp도 함께 맞춰주기(있을 때만)
            msg = shift_header_stamp_if_any(msg, new_t)

            out.write(topic, msg, t=new_t)
            wrote += 1

    print(f"  - 작성 완료: {wrote} msgs, 기준 0.000000s부터 정렬")


def main():
    # ros가 초기화 되어있지 않아도 rosbag은 동작하지만, rospy.Time 사용을 위해 init_node는 생략
    if not INPUT_BAGS:
        print("입력 목록이 비어 있습니다.")
        sys.exit(1)

    for path in INPUT_BAGS:
        try:
            process_one(path)
        except Exception as e:
            print(f"[ERROR] 처리 실패: {path} -> {e}")


if __name__ == "__main__":
    main()

