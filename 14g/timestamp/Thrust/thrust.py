#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
자동 처리 스크립트:
- 현재 폴더의 10g_*_timestamp.bag 파일들을 모두 찾아서
  /cf2/log1 (values: [0]=PWM, [1,2,3]=x,y,z of v_b = m a^B + R_W^B m g e3)
  /natnet_ros/cf2/pose (PoseStamped)
  /point_index4 (payload point)
  를 사용해 F_s(스프링), F_t(추력)을 Body 프레임, gf 단위로 계산하여
  <원본>_ftfs.bag 로 저장합니다.

생성 토픽:
- /ft/vec_gf  (geometry_msgs/Vector3Stamped)  : F_t^B [gf]
- /fs/vec_gf  (geometry_msgs/Vector3Stamped)  : F_s^B [gf]
- /ft/z_gf    (std_msgs/Float32)              : F_t,z [gf]
- /pwm        (std_msgs/Float32)              : PWM
"""

import os
import sys
import glob
import numpy as np
import rosbag
import rospy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32
from tf.transformations import quaternion_matrix

# --------- 설정 (필요시 아래 값만 수정) ----------
# 스프링 상수 모델: F_s = 0.5819 * (L - L0) + 0.0802
K_SPRING_A = 0.7686   # [N/m]
K_SPRING_B = 0.0448   # [N]
L0          = 0.205   # [m]  (영장 길이)

# log1은 이미 gf 단위임 → 변환하지 않음
VB_IS_GF = True

# 자동 탐지할 토픽 후보
LOG_TOPICS_CAND   = ["/cf2/log1", "/cf/log1"]
POSE_TOPICS_CAND  = ["/natnet_ros/cf2/pose", "/natnet/cf2/pose", "/natnet_ros/cf2/pose/pose"]
POINT_TOPICS_CAND = ["/point_index4", "/point/index4/points", "/point_index4/points"]

# 처리 대상 bag 패턴
INPUT_PATTERN = "14g_*_timestamp.bag"
# ---------------------------------------------------

GF_PER_NEWTON = 1.0 / 0.00980665  # ≈ 101.971621 gf/N

def quat_to_R_WB(qx, qy, qz, qw):
    """R_W^B: world->body. quaternion_matrix는 R_B^W 반환 -> 전치"""
    RBW = quaternion_matrix([qx, qy, qz, qw])[:3, :3]
    return RBW.T

def msg_time_ns(msg):
    return int(msg.header.stamp.to_nsec())

def nearest(samples, t_ns):
    """samples: 정렬된 [(t, payload), ...], t_ns와 가장 가까운 하나"""
    lo, hi = 0, len(samples)-1
    if hi < 0:
        return None
    while lo < hi:
        mid = (lo + hi) // 2
        if samples[mid][0] < t_ns:
            lo = mid + 1
        else:
            hi = mid
    idx = lo
    if idx > 0 and abs(samples[idx-1][0] - t_ns) < abs(samples[idx][0] - t_ns):
        idx -= 1
    return samples[idx]

def find_topic(bag, candidates):
    topics = set(bag.get_type_and_topic_info()[1].keys())
    for t in candidates:
        if t in topics:
            return t
    return None

def process_one_bag(path):
    print(f"\n[SCAN] {os.path.basename(path)}")
    with rosbag.Bag(path, "r") as bag:
        info = bag.get_type_and_topic_info()[1]
        topics = set(info.keys())
        for t in topics:
            print(f"  - {t}: {info[t].message_count} msgs")

        topic_log = find_topic(bag, LOG_TOPICS_CAND)
        topic_pose = find_topic(bag, POSE_TOPICS_CAND)
        topic_point = find_topic(bag, POINT_TOPICS_CAND)

        if not topic_log or not topic_pose or not topic_point:
            print("  ! required topics not found -> skip")
            return

        print(f"[USE] log : {topic_log}")
        print(f"[USE] pose: {topic_pose}")
        print(f"[USE] payl: {topic_point}")

        pwm_list, vb_list, pose_list, payl_list = [], [], [], []

        for topic, msg, _ in bag.read_messages():
            if topic == topic_log:
                vals = getattr(msg, "values", None)
                if vals is None or len(vals) < 4:
                    continue
                t = msg_time_ns(msg)
                pwm = float(vals[0])
                # ✅ values[1],[2],[3] = x, y, z 순서로 수정됨
                vb = np.array([float(vals[1]), float(vals[2]), float(vals[3])], dtype=float)
                pwm_list.append((t, pwm))
                vb_list.append((t, vb))
            elif topic == topic_pose:
                t = msg_time_ns(msg)
                p = msg.pose.position
                o = msg.pose.orientation
                p_q_W = np.array([p.x, p.y, p.z], dtype=float)
                R_WB = quat_to_R_WB(o.x, o.y, o.z, o.w)
                pose_list.append((t, p_q_W, R_WB))
            elif topic == topic_point:
                t = msg_time_ns(msg)
                if hasattr(msg, "points") and len(msg.points) > 0 and hasattr(msg.points[0], "x"):
                    p0 = msg.points[0]
                    p_p_W = np.array([p0.x, p0.y, p0.z], dtype=float)
                elif hasattr(msg, "pose"):
                    p_p_W = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)
                elif hasattr(msg, "point"):
                    p_p_W = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)
                else:
                    continue
                payl_list.append((t, p_p_W))

        if not pwm_list or not vb_list or not pose_list or not payl_list:
            print(f"  ! not enough data -> skip")
            return

        pwm_list.sort(); vb_list.sort(); pose_list.sort(); payl_list.sort()

        out_path = os.path.splitext(path)[0] + "_ftfs.bag"
        with rosbag.Bag(out_path, "w") as outbag:
            written = 0
            for t, pwm in pwm_list:
                nv = nearest(vb_list, t)
                np_ = nearest(pose_list, t)
                nd = nearest(payl_list, t)
                if nv is None or np_ is None or nd is None:
                    continue

                _, v_b = nv
                _, p_q_W, R_WB = np_
                _, p_p_W = nd

                v_b_gf = v_b.copy()

                # --- 스프링 힘 계산 ---
                s = p_q_W - p_p_W
                L = np.linalg.norm(s)
                if L < 1e-6:
                    continue
                uW = s / L

                # 스프링 힘 [N]
                Fs_N = K_SPRING_A * (L - L0) + K_SPRING_B
                if Fs_N < 0.0:
                    Fs_N = 0.0
                f_s_W_N = Fs_N * uW
                f_s_B_N = R_WB.dot(f_s_W_N)
                f_s_B_gf = f_s_B_N * GF_PER_NEWTON

                # 추력 (Body, gf)
                f_t_B_gf = v_b_gf + f_s_B_gf
                ft_z_gf = float(f_t_B_gf[2])

                stamp = rospy.Time.from_sec(t * 1e-9)

                vmsg_ft = Vector3Stamped()
                vmsg_ft.header.stamp = stamp
                vmsg_ft.vector.x, vmsg_ft.vector.y, vmsg_ft.vector.z = map(float, f_t_B_gf.tolist())
                outbag.write("/ft/vec_gf", vmsg_ft, stamp)

                vmsg_fs = Vector3Stamped()
                vmsg_fs.header.stamp = stamp
                vmsg_fs.vector.x, vmsg_fs.vector.y, vmsg_fs.vector.z = map(float, f_s_B_gf.tolist())
                outbag.write("/fs/vec_gf", vmsg_fs, stamp)

                msg_ftz = Float32(data=ft_z_gf)
                outbag.write("/ft/z_gf", msg_ftz, stamp)

                msg_pwm = Float32(data=float(pwm))
                outbag.write("/pwm", msg_pwm, stamp)

                written += 1

        print(f"[OK] wrote {written} samples -> {os.path.basename(out_path)}")

def main():
    rospy.init_node("make_ftfs_bags", anonymous=True, disable_signals=True)
    bag_files = sorted(glob.glob(INPUT_PATTERN))
    if not bag_files:
        print(f"No input bags matched '{INPUT_PATTERN}' in {os.getcwd()}")
        sys.exit(0)

    print("[TARGETS]")
    for f in bag_files:
        print(" -", f)

    for f in bag_files:
        try:
            process_one_bag(f)
        except Exception as e:
            print(f"[ERROR] {f}: {e}")

if __name__ == "__main__":
    main()