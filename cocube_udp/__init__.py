import socket
import time
import math
from threading import Thread
import uuid
import numpy as np


class CoCube:
    def __init__(self, robotID, gateway='192.168.3.1', local_ip='192.168.3.99', ip_prefix=100, udp_port=5000):
        self.robotID = robotID
        self.robot_ip = '.'.join(gateway.split('.')[:-1]) + f".{ip_prefix + robotID}"
        self.robot_port = udp_port + robotID
        self.localIP = local_ip
        self.local_port = udp_port
        try:
            self.sock_listen = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock_listen.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock_listen.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024)  # Send buffer
            self.sock_listen.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024)  # Receive buffer
            self.sock_listen.bind((self.localIP, self.robot_port))
            self.sock_listen.settimeout(3)

            self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock_send.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024)  # Send buffer
            self.sock_send.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)  # Receive buffer
            self.sock_send.settimeout(3)
        except Exception as e:
            print(f"Error: {e}")
            print("Fail to bind IP address，please check whether the IP address is correct！")
        self.pos_p = [0, 0]
        self.pos_m = [0, 0]
        self.yaw = 0
        self.angle = 0
        self.uuid_results = {}
        self.running = True
        self.data_process_thread = Thread(target=self.receive_data_thread)
        self.data_process_thread.start()
        print(f"agent_{robotID} initialize successfully!")

    def stop(self):
        self.running = False
        self.data_process_thread.join()
        self.sock_listen.close()
        self.sock_send.close()

    def __del__(self):
        self.stop()

    def receive_data_thread(self):
        '''
        Continuously receive messages from CoCube
        Params:
            None
        Returns:
            None
        '''
        while self.running:
            try:
                data, addr = self.sock_listen.recvfrom(1024)
                data = data.decode().split(',')
                if data[0].strip() == "res":            # Save the returned uuid and result into the dictionary
                    self.uuid_results[data[1].strip()] = data[2].strip()
                if len(self.uuid_results) > 100:         # Control dictionary length to avoid memory overflow
                    self.uuid_results = dict(list(self.uuid_results.items())[50:])
                elif data[0].strip() == "pos":
                    position = list(map(int, data[1:]))
                    self.pos_p = [position[0] / 64, position[1] / 64]
                    self.pos_m = [self.pos_p[0] *1.35*0.001, self.pos_p[1] *1.35*0.001]
                    self.angle = position[2]
                    self.yaw = position[2] * math.pi / 180 if position[2] < 180 else (position[2] - 360) * math.pi / 180
            except socket.timeout:
                print(f"No.{self.robotID}: Timeout occurred: No response in receive_positions.")
            except Exception as e:
                print(f"Error: {e}")
                break
        print(f"agent_{self.robotID} thread exit successfully!")

    def send_data(self, str_msg):
        message_bytes = str_msg.encode()
        self.sock_send.sendto(message_bytes, (self.robot_ip, self.local_port))

    def judge_whether_finished(self, uuid):
        if uuid in self.uuid_results.keys():
            result = self.uuid_results[uuid]
            del self.uuid_results[uuid]          # Remove uuids that have already been processed to avoid long dictionary lengths
            return True, result
        else:
            return False, None

    def process_data(self, func, params, block=False, timeout=3):
        '''
        Send the specified functions and parameters to CoCube and return uuid
        Params：
            func: function name
            params: parameters
            block: whether to block
            timeout: timeout
        Returns：
            uuid: unique identifier
        '''
        random_uuid = uuid.uuid4().hex[:6]

        block = str(int(block))
        if params:
            formatted_params = []
            for param in params:
                if isinstance(param, str):
                    formatted_params.append(f'"{param}"')
                else:
                    formatted_params.append(str(param))
            # Send the data to CoCube
            data_to_send = f"{block},{random_uuid},{func}," + ",".join(formatted_params)
        else:
            data_to_send = f"{block},{random_uuid},{func}"
        try:
            self.send_data(data_to_send)
        except Exception as e:
            print(f"Error: {e}")
            print("Fail to send data, please check whether the IP address is correct！")
        
        return random_uuid

     ###### CoCube Perception Interface ########
    
    def get_pos(self):
        return self.pos_p

    def get_pos_m(self):
        return self.pos_m

    def get_angle(self):
        return self.angle

    def get_yaw(self):
        return self.yaw    

    ###### CoCube Control Interface ########
    def move_millisecs(self, direction="forward", speed=40, duration=1000):
        if direction not in ["forward", "backward"]:
            raise ValueError("Direction must be forward or backward")
        if not 0 <= speed <= 50:
            raise ValueError("Speed must be in 0 - 50")
        direction = "cocube;" + direction
        return self.process_data(block=True, func="CoCube move for msecs", params=[direction, speed, duration],
                          timeout=3+duration/1000)

    def rotate_millisecs(self, direction="left", speed=40, duration=1000):
        if direction not in ["left", "right"]:
            raise ValueError("Direction must be left or right")
        if not 0 <= speed <= 50:
            raise ValueError("Speed must be in 0 - 50")
        direction = "cocube;" + direction
        return self.process_data(block=True, func="CoCube rotate for msecs", params=[direction, speed, duration],
                          timeout=3+duration/1000)

    def move(self, direction="forward", speed=40):
        if direction not in ["forward", "backward"]:
            raise ValueError("Direction must be forward or backward")
        if not 0 <= speed <= 50:
            raise ValueError("Speed must be in 0 - 50")
        direction = "cocube;" + direction
        return self.process_data(block=False, func="CoCube move", params=[direction, speed])

    def rotate(self, direction="left", speed=40):
        if direction not in ["left", "right"]:
            raise ValueError("Direction must be left or right")
        if not 0 <= speed <= 50:
            raise ValueError("Speed must be in 0 - 50")
        direction = "cocube;" + direction
        return self.process_data(block=False, func="CoCube rotate", params=[direction, speed])
    
    def set_wheel_speed(self, left_speed=40, right_speed=40):
        if not -50 <= left_speed <= 50 or not -50 <= right_speed <= 50:
            raise ValueError("Speed must be in -50 - 50")
        return self.process_data(block=False, func="CoCube set wheel", params=[int(left_speed), int(right_speed)])
    
    def wheels_stop(self):
        return self.process_data(block=False, func="CoCube wheels stop", params=[])

    def wheels_break(self):
        return self.process_data(block=False, func="CoCube wheels break", params=[])
    
    def move_by_steps(self, direction="forward", speed=40, step=50, block=True):
        if direction not in ["forward", "backward"]:
            raise ValueError("Direction must be forward or backward")
        if not 0 <= speed <= 50:
            raise ValueError("Speed must be in 0 - 50")
        direction = "cocube;" + direction
        return self.process_data(block=block, func="CoCube move by step", params=[direction, speed, step], timeout=60)
    
    def rotate_by_degree(self, direction="left", speed=40, degree=90, block=True):
        if direction not in ["left", "right"]:
            raise ValueError("Direction must be left or right")
        if not 0 <= speed <= 50:
            raise ValueError("Speed must be in 0 - 50")
        direction = "cocube;" + direction
        return self.process_data(block=block, func="CoCube rotate by degree", params=[direction, speed, degree], timeout=100)

    def rotate_to_angle(self, angle=0, speed=40, block=True):
        angle = angle % 360
        if not 0 <= speed <= 50:
            raise ValueError("Speed must be in 0 - 50")
        return self.process_data(block=block, func="CoCube rotate to angle", params=[angle, speed])
    
    def point_towards(self, target_x=0, target_y=0, speed=30, block=True):
        if not 0 <= speed <= 50:
            raise ValueError("Speed must be in 0 - 50")
        return self.process_data(block=block, func="CoCube point towards", params=[target_x, target_y, speed])

    def move_to_target(self, target_x=0, target_y=0, speed=40, block=True):
        if not 0 <= speed <= 50:
            raise ValueError("Speed must be in 0 - 50")
        return self.process_data(block=block, func="CoCube move to", params=[target_x, target_y, speed])

    ###### CoCube Display Interface ########
    def clear_display(self):
        return self.process_data(block=False, func="[tft:clear]", params=[])

    def set_pixel(self, x=0, y=0, color=(255, 255, 255)):
        if not all(0 <= x <= 240 for x in (x, y)):
            raise ValueError("X and Y must be in 0 - 240")
        if not all(0 <= x <= 255 for x in color):
            raise ValueError("RGB Value must be in 0 - 255")
        color = (color[0] << 16) | (color[1] << 8) | color[2]
        return self.process_data(block=False, func="[tft:setPixel]", params=[x, y, color])

    def fill_rect(self, x=0, y=0, width=10, height=10, color=(255, 255, 255)):
        if not all(0 <= x <= 240 for x in (x, y)):
            raise ValueError("X and Y must be in 0 - 240")
        if not all(0 <= x <= 255 for x in color):
            raise ValueError("RGB Value must be in 0 - 255")
        if not all(0 <= x <= 240 for x in (width, height)):
            raise ValueError("Width and Height must be in 0 - 240")
        color = (color[0] << 16) | (color[1] << 8) | color[2]
        return self.process_data(block=False, func="[tft:rect]", params=[x, y, width, height, color])
    
    def draw_text(self, text="Hello, CoCube!", x=10, y=10, color=(255, 255, 255)):
        if not all(0 <= x <= 240 for x in (x, y)):
            raise ValueError("X and Y must be in 0 - 240")
        if not all(0 <= x <= 255 for x in color):
            raise ValueError("RGB Value must be in 0 - 255")
        color = (color[0] << 16) | (color[1] << 8) | color[2]
        return self.process_data(block=False, func="tft_drawText", params=[text, x, y, color])
    
    def mb_display(self, matrix = np.ones((5, 5)), dtype=int):
        if matrix.shape != (5, 5):
            raise ValueError("Matrix must be 5x5")
        if not np.all((matrix == 0) | (matrix == 1)):
            raise ValueError("Matrix elements must be 0 or 1")
        weights = np.array([2**i for i in range(25)])
        code = matrix.flatten().dot(weights)
        return self.process_data(func="[display:mbDisplay]", params=[code], block="1")

    def set_display_color(self, color=(255, 255, 255)):
        if not all(0 <= x <= 255 for x in color):
            raise ValueError("RGB Value must be in 0 - 255")
        color = (color[0] << 16) | (color[1] << 8) | color[2]
        self.process_data(func="set display color", params=[color], block="0")

    def set_tft_backlight(self, brightness=5):
        if brightness not in range(0, 11):
            raise ValueError("brightness must be 0-10")
        return self.process_data(block=False, func="[tft:setBacklight]", params=[brightness])
        
    def draw_aruco_marker_on_tft(self, id=0):
        if id not in range(0, 100):
            raise ValueError("Aruco Marker ID must be in 0 - 99")
        return self.process_data(block=False, func="CoCube draw Aruco Marker on TFT", params=[id])

    def draw_apriltag_on_tft(self, id=0):
        if id not in range(0, 100):
            raise ValueError("AprilTag ID must be in 0 - 99")
        return self.process_data(block=False, func="CoCube draw AprilTag on TFT", params=[id])

    def call_bmp(self, bmp_name, x=0, y=0):
        return self.process_data(block=False, func="drawBMPfile", params=[bmp_name, x, y])

    def custom_func(self, func="", params=[], block=False):
        return self.process_data(block=block, func=func, params=params)
    
    ###### CoCube External Module Interface ########
    def power_on_module(self):
        return self.process_data(block=False, func="ccmodule_power on module", params=[])

    def gripper_open(self, block=True):
        return self.process_data(block=block, func="ccmodule_gripper open", params=[])

    def gripper_close(self, block=True):
        return self.process_data(block=block, func="ccmodule_gripper close", params=[])

    def gripper_degree(self, degree, block=True):
        if not 0 <= degree <= 70:
            raise ValueError("Degree must be in 0 - 70")
        return self.process_data(block=block, func="ccmodule_gripper degree", params=[degree])

    def set_NeoPixel_color(self, id, color):
        if id not in range(0, 49):
            raise ValueError("NeoPixel ID must be in 1 - 48")
        if not all(0 <= x <= 255 for x in color):
            raise ValueError("RGB Value must be in 0 - 255")
        color = (color[0] << 16) | (color[1] << 8) | color[2]
        return self.process_data(block=False, func="setNeoPixelColor", params=[id, color])

    def set_all_NeoPixels_color(self, color):
        if not all(0 <= x <= 255 for x in color):
            raise ValueError("RGB Value must be in 0 - 255")
        color = (color[0] << 16) | (color[1] << 8) | color[2]
        self.process_data(block=False, func="ccmodule_attach NeoPixels", params=[])
        time.sleep(0.1)
        return self.process_data(block=False, func="ccmodule_set all NeoPixels color", params=[color])

    def clear_NeoPixels(self):
        return self.process_data(block=False, func="clearNeoPixels", params=[])

    def ToF_distance(self):
        self.process_data(block=False, func="ccmodule_ToF connected", params=[])
        time.sleep(0.1)
        return self.process_data(block=True, func="ccmodule_ToF distance", params=[])
