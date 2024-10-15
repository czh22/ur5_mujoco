import numpy as np
import robosuite as suite
from robosuite import load_controller_config
import time

import utils.admittance_py as admittance_py
from ur_ikfast import ur_kinematics
import utils.imu_receiver as imu_receiver
import utils.force_feedback as force_feedback
import utils.camera2pos as camera2pos
from scipy.spatial.transform import Rotation


# 设置admittance控制参数
M = np.array([[20, 0, 0, 0, 0, 0],
              [0, 20, 0, 0, 0, 0],
              [0, 0, 20, 0, 0, 0],
              [0, 0, 0, 20, 0, 0],
              [0, 0, 0, 0, 20, 0],
              [0, 0, 0, 0, 0, 20]])
B = np.array([[200, 0, 0, 0, 0, 0],
              [0, 200, 0, 0, 0, 0],
              [0, 0, 200, 0, 0, 0],
              [0, 0, 0, 200, 0, 0],
              [0, 0, 0, 0, 200, 0],
              [0, 0, 0, 0, 0, 200]])
K = np.array([[180, 0, 0, 0, 0, 0],
              [0, 180, 0, 0, 0, 0],
              [0, 0, 180, 0, 0, 0],
              [0, 0, 0, 180, 0, 0],
              [0, 0, 0, 0, 180, 0],
              [0, 0, 0, 0, 0, 180]])
## 朝下
rotation = np.array([[-0.02185321, -0.99873381, -0.04531251],
                    [-0.99963531,  0.02254722, -0.01486196],
                    [ 0.01586481,  0.0449712,  -0.9988623]])

current_vel = np.array([0, 0, 0, 0, 0, 0])
current_pos = np.array([0.34870946, -0.01329686, 0.21732919, 3.1, 0, -1.6])
action = np.array([0, 0, 0, 0, 0, 0])


# Load the desired controller's default config as a dict
config = load_controller_config(default_controller='JOINT_POSITION')
config['kp'] = 50
config['output_max'] = 0.2
config['output_min'] = -0.2
config['interpolation'] = 'linear'

## 初始化相机位置获取
# cam2pos = camera2pos.cam2pos()
# time.sleep(5)

# create environment instance
env = suite.make(
    env_name="Wipe", # try with other tasks like "Stack" and "Door"
    robots="UR5e",  # try with other robots like "Sawyer" and "Jaco"
    has_renderer=True,
    has_offscreen_renderer=False,
    use_camera_obs=False,
    control_freq=50,
    controller_configs=config,
    horizon=2000,
    renderer="mujoco",
    # robot_configs=None,
)






# time.sleep(1)
# init_position = [0.34870946, -0.01329686, 0.21732919]
# first_position = cam2pos.get_pos()
# trans_position = [init_position[i] - first_position[i] for i in range(3)]
## 初始化ur5逆解
ur5e_arm = ur_kinematics.URKinematics('ur5e')

# get time step
time_step = 1.0 / env.control_freq
print(f"每一步仿真时间步长: {time_step} 秒")




while True:
    start_real_time = time.time()  # record real start time

    ## 视频释教的位置获取
    # position = cam2pos.get_pos()
    # position = [position[i] + trans_position[i] for i in range(3)]
    position = [0.44870946, -0.01329686, 0.11732919]
    # if env.sim.data.time > 10:
    #     position[2] = -0.05
   
    pose_rotation = Rotation.from_matrix(rotation)
    euler_angles = pose_rotation.as_euler('xyz')
    target_pos = np.concatenate((position, euler_angles), axis=0)

    torque = env.robots[0].ee_torque
    force = env.robots[0].ee_force
    end_force = np.concatenate((force, torque), axis=0)

    ## 计算feedback_force
    feedback_force = np.sqrt(np.sum(np.square(force)))
   

    # 限制力的大小
    for i in range(len(end_force)):
        if end_force[i] > 50:
            end_force[i] = 50
        elif end_force[i] < -50:
            end_force[i] = -50

    ## 计算admittance控制量
    current_pos, current_vel = admittance_py.admittance_pos(K, B, M, current_vel, current_pos, end_force, target_pos)
    # print(current_pos)
    # print(env.robots[0]._joint_positions)

    pose_rotation = Rotation.from_euler('xyz', current_pos[3:])
    matrix = pose_rotation.as_matrix()
    target_pos = np.concatenate((matrix, current_pos[:3, np.newaxis]), axis=1)
    
## 求解逆运动学               
    ctrl = ur5e_arm.inverse(target_pos, False, env.robots[0]._joint_positions)
  
    if ctrl is not None:
        action = ctrl - env.robots[0]._joint_positions
        print('inverse founded')
    
    obs, reward, done, info = env.step(action)  # take action in the environment
    env.render()  # render on display

    end_step_time = time.time()    # record step end time
    step_exec_time = end_step_time - start_real_time  # calculate step execution time
    
    # calculate needed delay time
    sleep_time = time_step - step_exec_time
    if sleep_time > 0:
        time.sleep(sleep_time)
    else:
        print(f"Warning: 执行时间{step_exec_time}超过时间步长 {time_step} 秒")

