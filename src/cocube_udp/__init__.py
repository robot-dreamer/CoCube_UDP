import socket
import time
from math import *
from threading import Thread
import uuid


class CoCube:
    def __init__(self, robotID, gateway='192.168.3.1', local_ip='192.168.3.3', ip_prefix=100, enable_return=True,
                 udp_port=5000):
        self.robotID = robotID
        self.robot_ip = '.'.join(gateway.split('.')[:-1]) + f".{ip_prefix + robotID}"
        self.robot_port = udp_port + robotID
        self.localIP = local_ip
        self.enable_return = enable_return
        self.local_port = udp_port
        self.sock_listen = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_listen.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024)  # 增加发送缓冲区大小
        self.sock_listen.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024)  # 增加接收缓冲区大小
        self.sock_listen.bind((self.localIP, self.robot_port))
        self.sock_listen.settimeout(3)

        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_send.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024)  # 增加发送缓冲区大小
        self.sock_send.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)  # 增加接收缓冲区大小
        self.sock_send.settimeout(3)
        self.pos_x = 0
        self.pos_y = 0
        self.pos_direction = 0
        self.result = 0
        self.uuid_func = {}
        self.current_uuid = ""
        data_process_thread = Thread(target=self.receive_data_thread)
        data_process_thread.start()
        print("Initial successfully!")

    def connected(self):
        if isinstance(self.message_general("position_X", [], debug=True, testConnct=True, feedback=True), str):
            print(f"robot {self.robotID} connected")
            return True
        else:
            print(f"robot {self.robotID} not connected")
            return False

    # push style, receive message continuously
    def receive_data_thread(self):
        while True:
            try:
                data, addr = self.sock_listen.recvfrom(1024)
                data = data.decode().split(',')
                if data[0].strip() == "res":
                    self.current_uuid = data[1].strip()
                    self.result = data[2].strip()
                elif data[0].strip() == "pos":
                    position = list(map(int, data[1:]))
                    self.pos_x = position[0] / 128
                    self.pos_y = position[1] / 128
                    self.pos_direction = position[2]
            except socket.timeout:
                print("Timeout occurred: No response in receive_positions.")
            except Exception as e:
                print(f"Error: {e}")
                break

    def send_data(self, str_msg):
        message_bytes = str_msg.encode()
        # print(f"send message: {str_msg}")
        self.sock_send.sendto(message_bytes, (self.robot_ip, self.local_port))

    def process_data(self, func, params, block, timeout=3):
        # send data
        random_uuid = uuid.uuid4().hex[:6]
        self.uuid_func[random_uuid] = func
        # 处理参数，将字符串加上引号
        if params:
            formatted_params = []
            for param in params:
                if isinstance(param, str):
                    formatted_params.append(f'"{param}"')
                else:
                    formatted_params.append(str(param))
            # 构造最终的消息字符串
            data_to_send = f"{block},{random_uuid},{func}," + ",".join(formatted_params)
        else:
            data_to_send = f"{block},{random_uuid},{func}"
        self.send_data(data_to_send)

        # wait for response
        if self.enable_return:
            start_time = time.time()
            while time.time() - start_time < timeout:
                if self.current_uuid == random_uuid:
                    print("now: " + self.current_uuid)
                    print(f"{self.uuid_func[random_uuid]} execute successfully!")
                    del self.uuid_func[random_uuid]
                    return True
            print(
                f"{self.uuid_func[random_uuid]} execute failed!,plz check your network connection or prolong the waiting time")
            return False

    def set_tft_backlight(self, flag=0):
        self.process_data(block="0", func="CoCube set TFT backlight", params=[flag])

    def draw_aruco_marker_on_tft(self, id=0):
        self.process_data(block="0", func="CoCube draw Aruco Marker on TFT", params=[id])

    def draw_apriltag_on_tft(self, id=0):
        self.process_data(block="0", func="CoCube draw AprilTag on TFT", params=[id])

    def move(self, direction="forward", speed=40):
        self.process_data(block="0", func="CoCube move", params=[direction, speed])

    def rotate(self, direction="left", speed=30):
        self.process_data(block="0", func="CoCube rotate", params=[direction, speed])

    def move_millisecs(self, direction='forward', speed=40, duration=1000):
        self.process_data(block="0", func="CoCube move for millisecs", params=[direction, speed, duration],
                          timeout=3 + duration / 1000)

    def rotate_millisecs(self, direction="left", speed=40, duration=1000):
        self.process_data(block="1", func="CoCube rotate for millisecs", params=[direction, speed, duration],
                          timeout=100)

    def move_by_steps(self, direction="forward", speed=40, step=50):
        self.process_data(block="1", func="CoCube move by step", params=[direction, speed, step], timeout=100)

    def rotate_by_degree(self, direction="left", speed=40, degree=90):
        self.process_data(block="1", func="CoCube rotate by degree", params=[direction, speed, degree], timeout=100)

    def set_wheel_speed(self, left_speed=40, right_speed=40):
        self.process_data(block="0", func="CoCube set wheel", params=[left_speed, right_speed])

    def wheels_stop(self):
        self.process_data(block="0", func="CoCube wheels stop", params=[])

    def wheels_break(self):
        self.process_data(block="0", func="CoCube wheels break", params=[])

    def rotate_to_angle(self, angle=0, speed=40):
        self.process_data(block="0", func="CoCube rotate to angle", params=[angle, speed])

    def rotate_to_target(self, target_x=0, target_y=0, speed=30):
        self.process_data(block="0", func="CoCube rotate to target", params=[target_x, target_y, speed])

    def move_to_target(self, target_x=0, target_y=0, speed=50):
        self.process_data(block="0", func="CoCube move to target", params=[target_x, target_y, speed])

    ###### CoCube External Module ########
    def power_on_module(self):
        self.process_data(block="0", func="Power on module", params=[])

    def power_off_module(self):
        self.process_data(block="0", func="Power off module", params=[])

    def gripper_open(self):
        self.process_data(block="0", func="Gripper Open", params=[])

    def gripper_close(self):
        self.process_data(block="0", func="Gripper Close", params=[])

    def gripper_degree(self, degree):
        # degree: -70-0
        self.process_data(block="0", func="Gripper degree", params=[degree])

    def attach_NeoPixel(self):
        self.process_data(block="0", func="attach NeoPixel", params=[])

    def set_all_NeoPixels_color(self, r, g, b):
        color = r << 16 | g << 8 | b
        self.process_data(block="0", func="set all NeoPixels color", params=[color])

    def clear_NeoPixels(self):
        self.process_data(block="0", func="clear NeoPixels", params=[])
