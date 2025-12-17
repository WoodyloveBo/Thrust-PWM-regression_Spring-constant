#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
/cf2/log1 토픽에서 timestamp 10~15초 구간의
평균 추력(values[3])과 평균 PWM(values[0])을 계산하는 스크립트.
"""

import rosbag
import rospy
import numpy as np
import glob
import os

# 처리 대상 bag 패턴
INPUT_PATTERN = "non_payload_T?_timestamp.bag"

# 분석 구간 (초)
START_T = 10.0
END_T   = 15.0

def compute_avg_from_bag(bag_path):
    print(f"\n[READ] {os.path.basename(bag_path)}")
    ft_vals, pwm_vals = [], []

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=["/cf2/log1"]):
            t_sec = msg.header.stamp.to_sec() if hasattr(msg, "header") else t.to_sec()
            if START_T <= t_sec <= END_T:
                try:
                    pwm_vals.append(float(msg.values[0]))   # PWM
                    ft_vals.append(float(msg.values[3]))    # F_t (gf)
                except (AttributeError, IndexError):
                    continue

    if not ft_vals or not pwm_vals:
        print("  ! Not enough data in range.")
        return None, None

    avg_ft = np.mean(ft_vals)
    avg_pwm = np.mean(pwm_vals)
    print(f"  - Mean(ft_z_gf) = {avg_ft:.4f} gf")
    print(f"  - Mean(PWM)     = {avg_pwm:.4f}")
    return avg_ft, avg_pwm

def main():
    bag_files = sorted(glob.glob(INPUT_PATTERN))
    if not bag_files:
        print(f"No files matched pattern '{INPUT_PATTERN}'")
        return

    results = []
    for f in bag_files:
        avg_ft, avg_pwm = compute_avg_from_bag(f)
        if avg_ft is not None:
            results.append((os.path.basename(f), avg_ft, avg_pwm))

    print("\n[SUMMARY 10~15s 평균값]")
    print(f"{'File':40s} {'Ft_mean(gf)':>15s} {'PWM_mean':>15s}")
    print("-" * 70)
    for fname, ft, pwm in results:
        print(f"{fname:40s} {ft:15.4f} {pwm:15.4f}")

if __name__ == "__main__":
    main()
