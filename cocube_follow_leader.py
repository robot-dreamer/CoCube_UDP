from cocube_udp import CoCube
import time
import math
import numpy as np
import yaml

def calculate_pos(leader_pos, follower_pos):
    distance = 75
    yaw = math.atan2(leader_pos[0] - follower_pos[0], leader_pos[1] - follower_pos[1])
    target_pos = [leader_pos[0] - distance * math.sin(yaw), leader_pos[1] - distance * math.cos(yaw)]
    return target_pos

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
    for i in range(num):
        color = np.random.randint(0, 256, size=(3))
        agents[i].set_display_color(color)
        agents[i].mb_display()

    """
    Each CoCube follows the previous CoCube and maintains a distance of 75 code points. 
    The direction is the connection direction between the current and the previous CoCube.
    """
    while True:
        for i in range(1, num):
            if np.linalg.norm(np.array([agents[i].pos_p[0], agents[i].pos_p[1]]) - np.array([agents[i-1].pos_p[0], agents[i-1].pos_p[1]])) < 0.1:
                continue
            target = calculate_pos([agents[i-1].pos_p[0], agents[i-1].pos_p[1]], [agents[i].pos_p[0], agents[i].pos_p[1]])
            agents[i].move_to_target(target[0], target[1])
        time.sleep(0.1)
    
