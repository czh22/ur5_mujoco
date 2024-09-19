import numpy as np
import time


current_time = last_time = time.time()
def admittance_pos(K, B, M, current_target_vel, current_target_pos, current_end_force, final_target_pos):
    """
    Compute the admittance control input for a given position reference.

    Parameters:
    {
    K (numpy.ndarray): Stiffness matrix.
    B (numpy.ndarray): Damping matrix.
    M (numpy.ndarray): Mass matrix,6x6.
    }：导纳控制参数

    current_target_pos: 当前的末端目标位置[x, y, z, α, β, γ].
    current_target_vel: 当前的末端目标速度[vx, vy, vz, ωx, ωy, ωz].
    current_target_accl: 当前的末端目标加速度[ax, ay, az, αx, αy, αz].
    current_end_force: 当前的末端力,注意将力传感器坐标系下的力转为世界坐标系下[fx, fy, fz, tx, ty, tz].
    final_target_pos: 最终的末端目标位置[x, y, z, α, β, γ].

    Returns:
    numpy.ndarray: Admittance control pos.
    """
    global current_time, last_time
    # Fext = Md*(ddxd-ddx0) + Dd*(dxd-dx0) + Kd*(xd-x0)
    # xd是当前末端的目标位置，x0是最终最初输入的目标位置，也是在没有外力阻碍下最终期望达到的位置
    current_target_accl = np.dot(M.T, (-current_end_force.T - np.dot(B, current_target_vel.T) - np.dot(K, (current_target_pos.T - final_target_pos.T))))

    for i in range(6):
        if current_target_accl[i] > 3:
            current_target_accl[i] = 3
        elif current_target_accl[i] < -3:
            current_target_accl[i] = -3

    current_time = time.time()
    dt = current_time - last_time
    current_target_vel = current_target_vel + current_target_accl * dt

    for i in range(6):
        if current_target_vel[i] > 0.6:
            current_target_vel[i] = 0.6 
        elif current_target_vel[i] < -0.6:
            current_target_vel[i] = -0.6
    current_target_pos = current_target_pos + current_target_vel * dt

    last_time = time.time()
    return current_target_pos, current_target_vel

    

