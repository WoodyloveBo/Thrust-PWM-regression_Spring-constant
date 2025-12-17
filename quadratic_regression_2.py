#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PWM-Thrust 2차 다항회귀 스크립트
- CSV 파일: PWM, Thrust
- 오름차순 정렬 후 2차 다항회귀 (Thrust = a*PWM^2 + b*PWM + c)
- 회귀식과 R² 출력, 시각화 그래프 포함
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import r2_score

# 1. CSV 불러오기
df = pd.read_csv("PWM-Thrust-alpha.csv")

# 2. 컬럼 이름 정리 및 오름차순 정렬
df.columns = df.columns.str.strip()
df = df.sort_values(by="PWM")

# 3. 데이터 추출
x = df["PWM"].values
y = df["Thrust"].values

# 4. 2차 다항 회귀 (np.polyfit)
coeffs = np.polyfit(x, y, 2)   # [a, b, c]
a, b, c = coeffs
y_pred = np.polyval(coeffs, x)
r2 = r2_score(y, y_pred)

# 5. 회귀 결과 출력
print("===== 2차 다항회귀 결과 =====")
print(f"Thrust = {a:.6e} * PWM^2 + {b:.6e} * PWM + {c:.6e}")
print(f"R² = {r2:.6f}")

# ===============================
# 6. 그래프 표시 (스타일 개선)
# ===============================

plt.figure(figsize=(8, 5))

plt.figure(figsize=(8, 5))

# 🔹 점 스타일 (Measurement point)
plt.scatter(
    x, y,
    s=50,                 
    color="#0072BD",
    alpha=0.9,
    label="PWM-Based Thrust"       # ← 변경됨
)

# ▶ X축 눈금: 30000, 35000, 40000 ...
x_min, x_max = plt.xlim()
x_start = int(np.floor(x_min / 5000) * 5000)
x_end   = int(np.ceil(x_max / 5000) * 5000)
plt.xticks(np.arange(x_start, x_end + 1, 5000))

# ▶ Y축 눈금: 40.0, 45.0, 50.0 ...
y_min, y_max = plt.ylim()
y_start = np.floor(y_min / 5) * 5
y_end   = np.ceil(y_max / 5) * 5
plt.yticks(np.arange(y_start, y_end + 0.01, 5.0))


# 🔹 회귀선 스타일
plt.plot(
    x, y_pred,
    linewidth=3.0,
    color="#D95319",
    label=f"Quadratic Fit (R²={r2:.4f})"     # ← 변경됨
)

plt.xlabel("PWM", fontsize=16)
plt.ylabel("Thrust (gf)", fontsize=16)
plt.legend(fontsize=14, frameon=False)
plt.tick_params(axis="both", labelsize=14)

plt.grid(True, linewidth=0.6, alpha=0.5)
plt.tight_layout()
plt.show()

