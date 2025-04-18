import numpy as np
import pinocchio as pin
import pandas as pd
import time
from pinocchio.visualize import MeshcatVisualizer

csv_path = "/home/robot/文档/vs_project/quadruped_mpc_6/solo_kinodynamics_result_xs.csv"

# 读取URDF文件和创建机器人模型
urdf_path = "/home/robot/文档/vs_project/quadruped_mpc_6/robot/galileo_mini_x5_description/galileo_mini_x5.urdf"
model = pin.buildModelFromUrdf(urdf_path)
package_dirs = ["/home/robot/文档/vs_project/quadruped_mpc_6/robot"]
visual_model = pin.buildGeomFromUrdf(model, urdf_path, pin.GeometryType.VISUAL, package_dirs=package_dirs)
collision_model = pin.buildGeomFromUrdf(model, urdf_path,pin.GeometryType.COLLISION, package_dirs=package_dirs)

# 设置可视化器
viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(loadModel=True)
viz.viewer.open()

# 读取轨迹数据
try:
    trajectory_data = pd.read_csv(csv_path, header=None)
    x_trajectory = trajectory_data.values
    q_trajectory = x_trajectory[:, :model.nq]
    
    while(True):
        for q in q_trajectory:
            viz.display(q)
            time.sleep(0.01)  # 可调整显示速度
        time.sleep(1)  # 可调整显示速度

        
except FileNotFoundError:
    print("找不到轨迹文件：trajectory_results.csv")
except Exception as e:
    print(f"发生错误：{str(e)}")