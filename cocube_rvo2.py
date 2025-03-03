# coding: utf-8
import rvo2
import math
import numpy as np
import time
import yaml
from cocube_udp import CoCube


# Limit the wheels' speed
def limit_speed(speed, inf, sup):
    if speed > sup:
        speed = sup
    if speed < inf:
        speed = inf
    return speed

class RVO2Robot(CoCube):
    def __init__(self, robotID, gateway='192.168.31.1', local_ip='192.168.31.169', ip_prefix=100, udp_port=5000):
        super().__init__(robotID=robotID, gateway=gateway, local_ip=local_ip, ip_prefix=ip_prefix, udp_port=udp_port)
        self.integral_ = 0
        self.prev_error_ = 0
        self.start_time = time.time()

    # PID Heading Controller
    def yawPIDController(self, target):
        k = 1.2
        kp = 1.8
        ki = 0.0
        kd = 0.5 
        # calculate the error
        error = self.rectify_angle(target - self.yaw)         # change to get_theta
        self.integral_ += error
        derivative = error - self.prev_error_

        control_input = k * (kp * error + ki * self.integral_ + kd * derivative)
        # update prev_error
        self.prev_error_ = error
        return control_input

    def rectify_angle(self, angle):
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # Calculate the control speed based on the target point
    def calculate_vel(self, target_pos):
        global_vel = [target_pos[0] - self.pos_m[1], target_pos[1] - self.pos_m[0], 0]
        global_vel_norm = global_vel / np.linalg.norm(global_vel)
        max_vel = 0.05
        target_vel = (global_vel_norm[0] * math.cos(self.yaw) + global_vel_norm[1] * math.sin(self.yaw)) * max_vel
        return target_vel

    def move2aim(self, target_pos, final_pos):
        ref_theta = math.atan2(target_pos[1] - self.pos_m[0], target_pos[0] - self.pos_m[1])
        vel = self.calculate_vel(target_pos)        # Calculate the partial velocity of the CoCube in its current orientation
        distance = [final_pos[0] - self.pos_m[1], final_pos[1] - self.pos_m[0]]
        # if close to the target, stop iterate
        if (np.linalg.norm(distance)) < 0.01:
            vel = 0
            yaw_vel = 0
        else:
            yaw_vel = self.yawPIDController(ref_theta)
        self.set_velocity(vel, yaw_vel)

    # Send control command
    def set_velocity(self, linear, angular):
        left_wheel_speed, right_wheel_speed = self.calculate_wheel_speeds(linear, angular)
        if left_wheel_speed == 0 and right_wheel_speed == 0:
            self.wheels_break()
        else:
            self.set_wheel_speed(left_wheel_speed, right_wheel_speed)

    def calculate_wheel_speeds(self, linear_velocity, angular_velocity, wheel_base=0.0355):
        """
        Calculate the speed of the left and right wheels

        :param linear_velocity: target linear velocity (m/s)
        :param _velocity: target angular velocity (rad/s)
        :param wheel_base: Distance between two wheels (m)
        :return: The speed of the left and right wheels (m/s)
        """
        right_wheel_speed = linear_velocity + (wheel_base * angular_velocity) / 2
        left_wheel_speed = linear_velocity - (wheel_base * angular_velocity) / 2
        right_wheel_speed = right_wheel_speed / (1.35*0.001) # Convert unit to map coordinates
        left_wheel_speed = left_wheel_speed / (1.35*0.001)

        left_wheel_speed = limit_speed(left_wheel_speed, -50, 50)
        right_wheel_speed = limit_speed(right_wheel_speed, -50, 50)

        return left_wheel_speed, right_wheel_speed

class Game:
    def __init__(self, robot_nums, rate = 10, gateway='192.168.31.1', local_ip='192.168.31.169',
                         ip_prefix=100, udp_port=5000):
        self.robots = [RVO2Robot(robotID=id+1, gateway=gateway, local_ip=local_ip, ip_prefix=ip_prefix,
                             udp_port=udp_port) for id in range(robot_nums)]
        self.final_positions = []
        self.interval = 1.0 / rate
        self.robot_nums = robot_nums
        # RVOSimulator(float timeStep, float neighborDist, size_t maxNeighbors,
        #           float timeHorizon, float timeHorizonObst, float radius,
        #           float maxSpeed, const Vector2 &velocity = Vector2())
        # neighborDist: Minimum distance between neighbors，maxNeighbors: max neighbors，

        self.sim = rvo2.PyRVOSimulator(1 / 2, 0.17, 3, 10, 4, 0.06, 0.1)     # change the maxSpeed,radius
        self.target_pos = []
        self.agents = ['a%s' % i for i in range(0, self.robot_nums)]

    # Initialize 
    def init_game(self):
        time.sleep(0.5)
        for i in range(self.robot_nums):
            # Set the coordinate system one into the right-hand coordinate system
            pos_m = [self.robots[i].pos_m[1], self.robots[i].pos_m[0]]
            self.agents[i] = self.sim.addAgent(tuple(self.robots[i].pos_m))     # 0,1,2 ...
            id = (i + int(self.robot_nums / 2)) % self.robot_nums
            self.final_positions.append([self.robots[id].pos_m[1], self.robots[id].pos_m[0]])   
        print("init_game successfully")

    def play(self):
        for i in range(10000):
            start_time = time.time()
            speed = 5
            times = 5
            for i in range(self.robot_nums):
                pos_m = [self.robots[i].pos_m[1], self.robots[i].pos_m[0]]
                self.sim.setAgentPosition(self.agents[i], tuple(pos_m))
            self.setAgentVelocity(v0=speed)
            self.Step(times)
            if(time.time() - start_time < self.interval):
                time.sleep(self.interval - (time.time() - start_time))

    # A local action calculation based on RVO2
    def Step(self, step=1):
        for j in range(step):
            self.sim.doStep()
            self.target_pos = [
                [round(self.sim.getAgentPosition(agent)[0], 3), round(self.sim.getAgentPosition(agent)[1], 3)]
                for agent in self.agents]
            for i, rob in enumerate(self.robots):
                rob.move2aim(self.target_pos[i], self.final_positions[i])
        return self.target_pos
    
    def stop(self):
        for robot in self.robots:
            robot.wheels_stop()

    def setAgentVelocity(self, v0):
        target_vel = []
        for i in range(self.robot_nums):
            target_vel.append(
                    ((self.final_positions[i][0] - self.robots[i].pos_m[1]) * v0,
                     (self.final_positions[i][1] - self.robots[i].pos_m[0]) * v0))
            self.sim.setAgentPrefVelocity(self.agents[i], target_vel[i])
        return target_vel

"""
Function: n CoCubes (n is even number), the target is the i-th CoCubes and 
the i+n/2 CoCubes interchange positions, real-time obstacle avoidance is achieved through 
the RVO2 algorithm. It can be adjusted by tweaking theNeighborDist and maxNeighbors parameters are 
used to adjust the obstacle avoidance effect, and the angle control uses a PID controller.
"""
if __name__ == "__main__":
    robot_names = 2
    with open('config.yml', 'r') as file:
        config = yaml.safe_load(file)
    
    gateway = config.get('gateway')
    hostip = config.get('hostip')
    ip_prefix = config.get('ip_prefix')
    port_prefix = config.get('port_prefix')

    rvo2game = Game(robot_names, gateway=gateway, local_ip=hostip, ip_prefix=ip_prefix, udp_port=port_prefix)
    time.sleep(0.5)
    
    try:
        rvo2game.init_game()
        rvo2game.play()
    except KeyboardInterrupt:
        print("Stopping all robots...")
        rvo2game.stop()
        print("All robots stopped.")
