import mujoco
import mujoco.viewer
import time
import numpy as np
from scipy.spatial.transform import Rotation
import admittance_py
from ur_ikfast import ur_kinematics
import matplotlib.pyplot as plt

# 设置admittance控制参数
M = np.array([[100, 0, 0, 0, 0, 0],
     [0, 100, 0, 0, 0, 0],
     [0, 0, 100, 0, 0, 0],
     [0, 0, 0, 100, 0, 0],
     [0, 0, 0, 0, 100, 0],
     [0, 0, 0, 0, 0, 100]])
B = np.array([[50, 0, 0, 0, 0, 0],
     [0, 50, 0, 0, 0, 0],
     [0, 0, 50, 0, 0, 0],
     [0, 0, 0, 50, 0, 0],
     [0, 0, 0, 0, 50, 0],
     [0, 0, 0, 0, 0, 50]])
K = np.array([[200, 0, 0, 0, 0, 0],
     [0, 200, 0, 0, 0, 0],
     [0, 0, 200, 0, 0, 0],
     [0, 0, 0, 200, 0, 0],
     [0, 0, 0, 0, 200, 0],
     [0, 0, 0, 0, 0, 200]])
current_vel = np.array([0, 0, 0, 0, 0, 0])
current_pos = np.array([0, 0, 0, 0, 0, 0])

# 初始化ur5逆解
ur5e_arm = ur_kinematics.URKinematics('ur5e')

model = mujoco.MjModel.from_xml_path('universal_robots_ur5e/scene.xml')
# model = mujoco.MjModel.from_xml_path('demo.xml')
# model.opt.gravity = 0
data = mujoco.MjData(model)
# 横着
# rotation = np.array([[-1, 0, 0],
#                      [0, 0, 1],
#                      [0, 1, 0]])
# 朝下
rotation = np.array([[-0,  1, 0],
                [ 1,  0, 0],
                [0, 0, -1]])

# 在主循环外部初始化列表来存储数据
time_points = []
feedback_forces = []

with mujoco.viewer.launch_passive(model, data) as viewer:
    position = [0.6, 0.1, 0.04]
    real_start_time = time.time()
    while viewer.is_running():
        
        
        if data.time > 10:
            position[2] = -0.03
       
        # 获取末端受力和力矩
        force = data.sensor('force_sensor').data
        torque = data.sensor('torque_sensor').data

                # 获取传感器的位置和旋转矩阵
        sensor_pos = data.site('attachment_site').xpos
        sensor_mat = data.site('attachment_site').xmat.reshape(3, 3)

                # 将力转换到世界坐标系
        force_world = sensor_mat.dot(force)

                # 将力矩转换到世界坐标系
        torque_world = sensor_mat.dot(torque)

                # 合并力和力矩
        end_force = np.concatenate((force_world, torque_world), axis=0)

                # print("世界坐标系中的力和力矩：", end_force)

                # 计算feedback_force
        feedback_force = np.sqrt(np.sum(np.square(force_world)))

        print("反馈力的平方和：", feedback_force)
## plt画图数据收集
        time_points.append(data.time)
        feedback_forces.append(feedback_force)
       

        if position is not None:

            pose = np.concatenate((rotation, np.array(position)[:, np.newaxis]), axis=1)
          

                # 释教姿态3x4转6
            pose_rotation = Rotation.from_matrix(pose[:, :3])
            euler_angles = pose_rotation.as_euler('xyz')
            target_pos = np.concatenate((pose[:, -1], euler_angles), axis=0)

            end_pose = np.concatenate(
                    (data.site('attachment_site').xmat.reshape(3, 3), data.site('attachment_site').xpos[:, np.newaxis]), axis=1)
            print(end_pose)
 
                # end_rotation = Rotation.from_matrix(end_pose[:, :-1])
                # end_euler_angles = end_rotation.as_euler('xyz')
                # current_pos = np.concatenate((end_pose[:, -1], end_euler_angles), axis=0)


# 限制力的大小
            for i in range(len(end_force)):
                if end_force[i] > 50:
                    end_force[i] = 50
                elif end_force[i] < -50:
                    end_force[i] = -50
                # 计算admittance控制量  
            current_pos, current_vel = admittance_py.admittance_pos(K, B, M, current_vel, current_pos, end_force, target_pos)

            pose_rotation = Rotation.from_euler('xyz', current_pos[3:])
            matrix = pose_rotation.as_matrix()
            target_pos = np.concatenate((matrix, current_pos[:3, np.newaxis]), axis=1)
            # print(target_pos)
                # print(data.qpos.shape)
            ctrl = ur5e_arm.inverse(target_pos, False, data.qpos[-6:])
                # print(ctrl)
            # 设置控制量
            if ctrl is not None:
                print(True)
                data.ctrl[-6:] = ctrl
# 进行仿真
        current_time = time.time()
        if current_time - real_start_time < data.time:
            time.sleep(data.time - (current_time - real_start_time))
        mujoco.mj_step(model, data, 10)
        viewer.sync()

        # print(time.time() - start_time)

# 主循环结束后绘制图表
plt.figure(figsize=(20, 300))
plt.plot(time_points, feedback_forces)
plt.title('Feedback Force 随时间变化')
plt.xlabel('时间 (秒)')
plt.ylabel('Feedback Force')
plt.grid(True)
plt.show()
