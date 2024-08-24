from cocube_udp import CoCube

import time
import random

if __name__ == '__main__':
    robot1 = CoCube(4, enable_return=False)
    robot1.process_data(block="0", func="[display:mbPlot]", params=[3, 3])
    while True:
        robot1.process_data(func="[display:mbSetColor]", params=[16711680], block='0')
        time.sleep(1)
        robot1.process_data(func="[display:mbSetColor]", params=[65280], block='0')
        time.sleep(1)
        robot1.move("forward", 50)
        time.sleep(2)
        robot1.move_by_steps("backward", 50, 50)
        time.sleep(3)
        robot1.wheels_stop()
        time.sleep(0.1)
