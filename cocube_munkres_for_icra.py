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

# Unit from mm to map coordinates
def preprocess_data(points):
    for i in range(len(points)):
        points[i] = [(points[i][0]-10) / 1.35, points[i][1] / 1.35]
    print(points)
    return points

def rotate_angle_thread(agent):
    time.sleep(0.3)
    agent.rotate_to_angle(180)

def process_munkres(agents, pos, color_class, color):
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
    time.sleep(1)

    for robot_id, pos_id in order.items():
        if(pos_id < color_class[0]):
            agents[robot_id].set_display_color(color[0][0], color[0][1], color[0][2])
            agents[robot_id].mb_display()
        elif(pos_id < color_class[1]):
            agents[robot_id].set_display_color(color[1][0], color[1][1], color[1][2])
            agents[robot_id].mb_display()
        elif(pos_id < color_class[2]):
            agents[robot_id].set_display_color(color[2][0], color[2][1], color[2][2])
            agents[robot_id].mb_display()
        else:
            agents[robot_id].set_display_color(color[3][0], color[3][1], color[3][2])
            agents[robot_id].mb_display()

if __name__ == "__main__":
    num = 32

    with open('config.yml', 'r') as file:
        config = yaml.safe_load(file)
    
    wifi_name = config.get('wifi_name')
    password = config.get('password')
    gateway = config.get('gateway')
    hostip = config.get('hostip')
    netmask = config.get('netmask')
    ip_prefix = config.get('ip_prefix')
    port_prefix = config.get('port_prefix')

    agents = [CoCube(id+1, gateway=gateway, local_ip=hostip, 
        ip_prefix=ip_prefix, udp_port=port_prefix) for id in range(num)]
    time.sleep(0.5)
    # Set the location in advance, "ICRA" first and then "2025"
    icra_pos = [(80, 110), (181, 110), (281, 110), (181, 191), (181, 272),(181, 361), (181, 435),(92, 435), (278, 435),
     (506, 132), (400,152), (330, 230), (320, 326), (400, 408),(505, 420),
     (620, 113), (620, 223), (620, 332), (620, 439), (722, 89), (804, 171), (727,241),(723, 352), (797, 431),
     (1016, 112), (961, 221), (1090, 218), (924, 326), (1024, 324), (1130, 323),(899, 430), (1166, 430)]
    _2025_pos = [(61,182),(148,105), (236,182), (205,265),(140,349),(69,435),(166,435),(260,435),
                 (336,231),(336,327),(427,125),(427,433),(518,231),(518,327),
                 (605, 182), (692, 105), (780, 182), (749, 265), (684, 349), (613, 435), (710, 435), (804, 435),
                 (1128,102),(1043,102),(959,102),(950,185),(940,269),(1031,284),(1114,311),(1114,406),(1023,448),(925,448)
                 ]
    # Unit from mm to map coordinates
    icra_pos = preprocess_data(icra_pos)
    _2025_pos = preprocess_data(_2025_pos)
    # Set the display color of CoCube corresponding to each group of letters
    icra_color = [[110, 43, 127], [242, 167, 59], [110, 43, 127], [110, 43, 127]]
    _2025_color = [[235,111,59], [255,255,59], [0,196,139], [14,97,255]]

    for i in range(num):
        agents[i].clear_display()
    time.sleep(1)

    input("***input anything to continue!*** : ")
    process_munkres(agents, icra_pos,[9, 15, 24], icra_color)
    input("***input anything to continue!*** : ")
    process_munkres(agents, _2025_pos,[8, 14, 22], _2025_color)