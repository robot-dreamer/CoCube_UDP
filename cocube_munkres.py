import numpy as np
import time
from concurrent.futures import ThreadPoolExecutor
import yaml

from cocube_udp import CoCube
from munkres import Munkres


def distanceMatrix(current_pos, target_pos):
    matrix = []
    for i in current_pos:
        distance = []
        for j in target_pos:
            dis = np.linalg.norm(np.array(i) - np.array(j))
            distance.append(dis)
        matrix.append(distance)
    return matrix

def munkresMatrix(dis_matrix):
    order = {}
    m = Munkres()
    indexes = m.compute(dis_matrix)
    total = 0
    for row, column in indexes:
        value = dis_matrix[row][column]
        total += value
        print(f'({row}, {column}) -> {value}')
        order[row] = column
    print('total distance: ', total)
    return order

def rotate_angle_thread(agent):
    time.sleep(0.3)
    agent.rotate_to_angle(180)

def process_munkres(agents, pos):
    num = len(agents)
    finish_flag = [False] * num
    executor = ThreadPoolExecutor(max_workers=len(agents))
    time.sleep(0.5)
    current_pos = []
    for i in range(num):
        current_pos.append([agents[i].pos_p[0], agents[i].pos_p[1]])

    matrix = distanceMatrix(current_pos, pos)
    order = munkresMatrix(matrix)

    time.sleep(1)
    for robot_id, pos_id in order.items():
        agents[robot_id].move_to_target(pos[pos_id][0], pos[pos_id][1])
        print(f"robot{robot_id} move to target position: {pos[pos_id][0], pos[pos_id][1]}")
    # Use thread pools to allow each CoCUbe to rotate independently after it arrives to avoid blockage
    while True:
        for robot_id, pos_id in order.items():
            dis = [pos[pos_id][0]-agents[robot_id].pos_p[0], pos[pos_id][1]-agents[robot_id].pos_p[1]]
            if np.linalg.norm(dis) < 10 and not finish_flag[robot_id]:
                executor.submit(rotate_angle_thread, (agents[robot_id]))
                finish_flag[robot_id] = True
        if all(finish_flag):
            break
        time.sleep(0.05)
    time.sleep(2)
    for i in range(num):
        agents[i].wheels_break()
    print("All robots have reached the target position!")
    for i in range(len(agents)):
        color = np.random.randint(0, 256, size=(3))
        agents[i].set_display_color(color)
        agents[i].mb_display()

if __name__ == "__main__":
    num = 2
    with open('config.yml', 'r') as file:
        config = yaml.safe_load(file)
    
    gateway = config.get('gateway')
    hostip = config.get('hostip')
    ip_prefix = config.get('ip_prefix')
    port_prefix = config.get('port_prefix')

    agents = [CoCube(id+1, gateway=gateway, local_ip=hostip, 
        ip_prefix=ip_prefix, udp_port=port_prefix) for id in range(num)]

    time.sleep(0.5)
    # target points
    target_pos = [(80, 110), (181, 110)]

    for i in range(num):
        agents[i].clear_display()
    time.sleep(1)
    aa = input("***input anything to start!*** : ")
    process_munkres(agents, target_pos)