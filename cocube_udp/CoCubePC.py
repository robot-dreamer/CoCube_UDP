import math

import CoCubeUDP
import time
import random

if __name__ == '__main__':
    robot1 = CoCubeUDP.CoCubeUDP(1, enable_return=False)
    # robot2 = CoCubeUDP.CoCubeUDP(2)
    # robot1.move_millisecs("forward",50,1000)
    # robot2.wheels_break()
    while True:
        x = random.randint(1, 5)
        y = random.randint(1, 5)
        robot1.process_data(block="0", func="[display:mbPlot]", params=[x, y])
        time.sleep(0.1)
        robot1.process_data(block="0", func="[display:mbUnplot]", params=[x, y])
        time.sleep(0.1)


