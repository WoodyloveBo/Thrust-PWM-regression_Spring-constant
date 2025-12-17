#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
distance_autodetect_v3.py
- /natnet_ros/cf2/pose 와 /point_index4 자동 감지
- pose-point 벡터와 노름 계산 + 단위벡터, 평균길이 편차, dt 출력
- 콘솔에 요약 통계도 출력
"""

import os, csv, math
import rosbag
from bisect import bisect_left
from statistics import mean, pstdev

JOBS = [
    {"bag": "19g_T1_timestamp.bag", "start": 15.0, "end": 20.0, "tol": 0.02, "out": None},
    {"bag": "19g_T2_timestamp.bag", "start": 15.0, "end": 20.0, "tol": 0.02, "out": None},
    {"bag": "19g_T3_timestamp.bag", "start": 15.0, "end": 20.0, "tol": 0.02, "out": None},
]

POSE_TOPIC_CANDIDATES = [
    "/natnet_ros/cf2/pose/pose",
    "/natnet_ros/cf2/pose",
    "/natnet/cf2/pose/pose",
    "/natnet/cf2/pose",
    "/cf2/pose",
]

POINT_TOPIC_CANDIDATES = [
    "/point_index4",
    "/point_index4/points",
    "/point/index4/points",
    "/point/index4",
]

def sec_ros_time(t):
    try:
        return float(t.to_sec())
    except Exception:
        return None

def time_of_msg(topic, msg, t):
    header = getattr(msg, "header", None)
    if header and hasattr(header, "stamp"):
        ts = sec_ros_time(header.stamp)
        if ts is not None and ts > 0:
            return ts
    return sec_ros_time(t)

def list_topics(bag_path):
    topics = {}
    with rosbag.Bag(bag_path, "r") as bag:
        for conn in bag._get_connections():
            topics[conn.topic] = topics.get(conn.topic, 0) + bag.get_message_count(conn.topic)
    return topics

def pick_existing_topic(bag_path, candidates):
    with rosbag.Bag(bag_path, "r") as bag:
        existing = {conn.topic for conn in bag._get_connections()}
    for cand in candidates:
        if cand in existing:
            return cand
    return None

def extract_pose_xyz(msg):
    if hasattr(msg, "pose") and hasattr(msg.pose, "pose") and hasattr(msg.pose.pose, "position"):
        p = msg.pose.pose.position;  return (p.x, p.y, p.z)
    if hasattr(msg, "pose") and hasattr(msg.pose, "position"):
        p = msg.pose.position;       return (p.x, p.y, p.z)
    if hasattr(msg, "position"):     # geometry_msgs/Pose
        p = msg.position;            return (p.x, p.y, p.z)
    if hasattr(msg, "transform") and hasattr(msg.transform, "translation"):
        t = msg.transform.translation; return (t.x, t.y, t.z)
    return None

def extract_point_xyz(msg):
    if hasattr(msg, "points"):          # sensor_msgs/PointCloud
        try:
            pt = msg.points[0]; return (pt.x, pt.y, pt.z)
        except Exception:
            pass
    if hasattr(msg, "point"):           # geometry_msgs/PointStamped
        pt = msg.point; return (pt.x, pt.y, pt.z)
    if all(hasattr(msg, a) for a in ("x","y","z")):  # geometry_msgs/Point
        return (msg.x, msg.y, msg.z)
    if hasattr(msg, "data") and isinstance(msg.data, (list,tuple)) and len(msg.data) >= 3:
        try:
            return (float(msg.data[0]), float(msg.data[1]), float(msg.data[2]))
        except Exception:
            pass
    return None

def probe_first_xyz(bag_path, topic, extractor, max_probe=30):
    with rosbag.Bag(bag_path, "r") as bag:
        k = 0
        for _, msg, _ in bag.read_messages(topics=[topic]):
            xyz = extractor(msg)
            if xyz is not None:
                return xyz
            k += 1
            if k >= max_probe:
                break
    return None

def nearest_idx(times, t):
    i = bisect_left(times, t)
    if i == 0: return 0
    if i == len(times): return len(times)-1
    return i-1 if (t - times[i-1]) <= (times[i] - t) else i

def default_out_path(bag_path, start, end):
    base, _ = os.path.splitext(bag_path)
    return f"{base}_seg_{start:.3f}_{end:.3f}.csv"

def save_csv(path, rows):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "t",
                "pose_x","pose_y","pose_z",
                "point_x","point_y","point_z",
                "dx","dy","dz","norm","dt",
                "ux","uy","uz","l","l_err"
            ]
        )
        writer.writeheader()
        for r in rows:
            writer.writerow(r)

def read_and_sync(bag_path, pose_topic, point_topic, start, end, tol):
    pose_t, pose_v = [], []
    point_t, point_v = [], []

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[pose_topic, point_topic]):
            ts = time_of_msg(topic, msg, t)
            if ts is None or ts < start or ts > end:
                continue
            if topic == pose_topic:
                pq = extract_pose_xyz(msg)
                if pq is not None:
                    pose_t.append(ts); pose_v.append(pq)
            else:
                pp = extract_point_xyz(msg)
                if pp is not None:
                    point_t.append(ts); point_v.append(pp)

    rows = []
    if not pose_t or not point_t:
        return rows

    # 최근접 매칭 + 벡터/노름 계산
    tmp_rows = []
    for tq, pq in zip(pose_t, pose_v):
        j = nearest_idx(point_t, tq)
        tp = point_t[j]
        if abs(tp - tq) <= tol:
            pp = point_v[j]
            dx = pq[0]-pp[0]; dy = pq[1]-pp[1]; dz = pq[2]-pp[2]
            l = math.sqrt(dx*dx + dy*dy + dz*dz)
            if l > 0:
                ux, uy, uz = dx/l, dy/l, dz/l
            else:
                ux=uy=uz=0.0
            tmp_rows.append({
                "t": tq,
                "pose_x": pq[0], "pose_y": pq[1], "pose_z": pq[2],
                "point_x": pp[0], "point_y": pp[1], "point_z": pp[2],
                "dx": dx, "dy": dy, "dz": dz, "norm": l,
                "dt": tp - tq,
                "ux": ux, "uy": uy, "uz": uz,
                "l": l,  # same as norm (alias)
            })

    # 평균 길이 편차 l_err 추가
    if tmp_rows:
        mL = mean([r["l"] for r in tmp_rows])
        for r in tmp_rows:
            r["l_err"] = r["l"] - mL

    return tmp_rows

if __name__ == "__main__":
    for job in JOBS:
        bag  = job["bag"]
        start= float(job["start"]); end=float(job["end"])
        tol  = float(job.get("tol", 0.02))
        out  = job.get("out", None)

        print(f"\n[SCAN] {bag}")
        try:
            topics = list_topics(bag)
        except Exception as e:
            print(f"  ! bag open failed: {e}")
            continue
        for k, v in sorted(topics.items()):
            print(f"  - {k}: {v} msgs")

        pose_topic  = pick_existing_topic(bag, POSE_TOPIC_CANDIDATES)
        point_topic = pick_existing_topic(bag, POINT_TOPIC_CANDIDATES)
        if not pose_topic or not point_topic:
            print(f"  ! pose_topic={pose_topic}, point_topic={point_topic} (둘 중 하나 없음) -> 후보 보강 필요")
            continue

        pose_sample  = probe_first_xyz(bag, pose_topic,  extract_pose_xyz)
        point_sample = probe_first_xyz(bag, point_topic, extract_point_xyz)
        print(f"[USE] pose:  {pose_topic}  (sample={pose_sample})")
        print(f"[USE] point: {point_topic} (sample={point_sample})")

        rows = read_and_sync(bag, pose_topic, point_topic, start, end, tol)

        # 요약 통계
        if not rows:
            print("  ! 동기화된 쌍이 없습니다.")
            continue

        L = [r["l"] for r in rows]
        DT = [abs(r["dt"]) for r in rows]
        mL = mean(L); sL = pstdev(L) if len(L) > 1 else 0.0
        mDT = mean(DT)
        print(f"[SYNC] pairs={len(rows)} tol={tol}s")
        print(f"[STAT] length mean={mL:.6f} std={sL:.6f} min={min(L):.6f} max={max(L):.6f}")
        print(f"[STAT] |dt| mean={mDT:.6f} s, max={max(DT):.6f} s")

        if out is None:
            out = default_out_path(bag, start, end)
        save_csv(out, rows)
        print(f"[SAVE] {out}")
