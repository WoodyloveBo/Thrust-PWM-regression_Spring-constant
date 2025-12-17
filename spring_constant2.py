import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator, FormatStrFormatter
from sklearn.linear_model import LinearRegression

# ===============================
# 1) 데이터 로드 & 컬럼 정리
# ===============================
df = pd.read_csv("spring_distance.csv", encoding="utf-8-sig")
df.columns = df.columns.str.strip().str.replace('\ufeff', '', regex=False)
df = df.replace([np.inf, -np.inf], np.nan).dropna(subset=["Weight", "displacement"])

# ===============================
# 2) 단위 변환 & 보정
# ===============================
df["Force_N"] = df["Weight"].astype(float) / 1000.0 * 9.81
df["x_corr"]  = df["displacement"].astype(float) - 0.205

# ===============================
# 3) 회귀 (F = kx + b)
# ===============================
X = df[["x_corr"]].values
y = df["Force_N"].values

model = LinearRegression()
model.fit(X, y)

k = float(model.coef_[0])
b = float(model.intercept_)
n = len(X)

# Pearson r
x = df["x_corr"].values
r = np.corrcoef(x, y)[0, 1]

# 회귀선용 x
x_fit = np.linspace(X.min(), X.max(), 300).reshape(-1, 1)
y_fit = model.predict(x_fit)

# ===============================
# 4) 플롯 (CI 없음)
# ===============================
plt.rcParams["figure.dpi"]  = 150
plt.rcParams["savefig.dpi"] = 600

fig = plt.figure(figsize=(6.2, 4.6))
ax = plt.gca()

# --- 축 눈금 간격 설정 (0.05 step) ---
ax.xaxis.set_major_locator(MultipleLocator(0.05))
ax.xaxis.set_major_formatter(FormatStrFormatter("%.2f"))

ax.yaxis.set_major_locator(MultipleLocator(0.05))
ax.yaxis.set_major_formatter(FormatStrFormatter("%.2f"))
# ---------------------------------------

# ===============================
# 🔵 Measurement 점 스타일 지정
# ===============================
plt.scatter(
    df["x_corr"], df["Force_N"],
    s=40,                    # ⬅ 점 크기 증가
    alpha=0.9,
    color="#0072BD",         # ⬅ 점 색 (파란색)
    label="Spring-Based Force"
)

# ===============================
#  축 범위 설정
# ===============================
x_min, x_max = df["x_corr"].min(), df["x_corr"].max()
y_min, y_max = df["Force_N"].min(), df["Force_N"].max()
padx = 0.10 * (x_max - x_min if x_max > x_min else 1.0)
pady = 0.10 * (y_max - y_min if y_max > y_min else 1.0)

x_left  = min(x_min - padx, 0.0)
x_right = max(x_max + padx, 0.0)
y_bot   = min(y_min - pady, 0.0)
y_top   = max(y_max + pady, 0.0)

plt.xlim(x_left, x_right)
plt.ylim(y_bot, y_top)

# ===============================
# 🔴 회귀선 스타일 지정
# ===============================
xmin, xmax = ax.get_xlim()
xx_line = np.array([xmin, xmax])
yy_line = k * xx_line + b

plt.plot(
    xx_line, yy_line,
    linewidth=2.5,           # ⬅ 선 두께 증가
    color="#D95319",         # ⬅ 회귀선 색 (주황/빨강)
    label=r"Linear Model : $F = kx + b$"
)

# (0,0) 표시
plt.axhline(0, linewidth=0.8)
plt.axvline(0, linewidth=0.8)

# 라벨 크기
plt.xlabel(r"Spring extension $L-L_0$ (m)", fontsize=15)
plt.ylabel(r"Force (N)", fontsize=15)

# 범례 크기 증가
plt.legend(loc="upper left", frameon=False, fontsize=14, handlelength=3.0)

plt.grid(True, linewidth=0.6, alpha=0.5)
plt.tick_params(axis="both", which="major", labelsize=14)
plt.tight_layout()

# 결과값 박스
eq_text = (rf"$k = {k:.4f}\ \mathrm{{N/m}}$" + "\n"
           rf"$b = {b:.4f}\ \mathrm{{N}}$" + "\n"
           rf"$R = {r:.4f}$")

plt.annotate(
    eq_text,
    xy=(0.98, 0.02), xycoords="axes fraction",
    ha="right", va="bottom", fontsize=11
)

# 저장/출력
plt.show()
plt.savefig("spring_constant_test5.png", bbox_inches="tight")

print(f"k = {k:.6f} N/m")
print(f"b = {b:.6f} N")
print(f"R (Pearson) = {r:.6f}, n = {n}")
