import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import Polygon

# 文件路径
zmp_pos_csv = "/home/robot/文档/vs_project/Quadruped-Robot/Quadruped-Robot-3/solo_kinodynamics_result_zmp_pos.csv"
foot_pos_csv = "/home/robot/文档/vs_project/Quadruped-Robot/Quadruped-Robot-3/solo_kinodynamics_result_feet_pos.csv"
contact_state_csv = "/home/robot/文档/vs_project/Quadruped-Robot/Quadruped-Robot-3/solo_kinodynamics_result_contact_state.csv"

# 读取数据
zmp_data = pd.read_csv(zmp_pos_csv, header=None).values
feet_data = pd.read_csv(foot_pos_csv, header=None).values
contact_data = pd.read_csv(contact_state_csv, header=None).values

num_frames = min(len(zmp_data), len(feet_data), len(contact_data))

# # 初始化绘图
# fig = plt.figure()
# ax = fig.add_subplot(111, projection="3d")

# def update3D(frame):
#     ax.cla()

#     # 设置坐标轴
#     ax.set_xlim([-1, 0.5])
#     ax.set_ylim([-0.5, 0.5])
#     ax.set_zlim([0, 0.5])
#     ax.set_xlabel("X")
#     ax.set_ylabel("Y")
#     ax.set_zlabel("Z")
#     ax.set_title(f"Frame {frame}")

#     feet = feet_data[frame].reshape(4, 3)
#     contact = contact_data[frame]

#     # 绘制足端点
#     for i, (pos, c) in enumerate(zip(feet, contact)):
#         color = "g" if c == 1 else "r"
#         ax.scatter(pos[0], pos[1], pos[2], color=color, s=50)

#     # 绘制接触多边形
#     contact_feet = [feet[i] for i in range(4) if contact[i] == 1]
#     if len(contact_feet) >= 2:
#         poly = np.array(contact_feet)
#         verts = [poly[:, :3]]
#         ax.add_collection3d(
#             Poly3DCollection(verts, facecolors="cyan", linewidths=1, alpha=0.3)
#         )

#     # 绘制ZMP点
#     zmp = zmp_data[frame]
#     ax.scatter(zmp[0], zmp[1], zmp[2], color="blue", s=100, label="ZMP")

fig, ax = plt.subplots()
ax.set_aspect('equal')

def update2D(frame):
    ax.clear()
    ax.set_xlim([-1, 0.5])
    ax.set_ylim([-0.5, 0.5])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title(f"XY Projection - Frame {frame}")

    feet = feet_data[frame].reshape(4, 3)
    contact = contact_data[frame]

    # 绘制足端（XY）
    for i, (pos, c) in enumerate(zip(feet, contact)):
        color = "g" if c == 1 else "r"
        ax.scatter(pos[0], pos[1], color=color, s=50)

    # 接触多边形（XY投影）
    contact_feet = [feet[i][:2] for i in range(4) if contact[i] == 1]
    if len(contact_feet) >= 2:
        poly = Polygon(
            contact_feet, closed=True, facecolor="cyan", alpha=0.3, edgecolor="black"
        )
        ax.add_patch(poly)

    # 绘制 ZMP 点（XY）
    zmp = zmp_data[frame]
    ax.scatter(zmp[0], zmp[1], color="blue", s=100, label="ZMP")
    ax.legend(loc="upper right")


ani = animation.FuncAnimation(fig, update2D, frames=num_frames, interval=50)
# plt.show()
# 保存为 MP4 文件
mp4_filename = "quadruped_zmp_motion_fly_high.mp4"
ani.save(mp4_filename, writer='ffmpeg', fps=20)
print(mp4_filename + " saved successfully.")
