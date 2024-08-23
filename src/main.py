from src.cocube_udp import CoCube

import time
import random

if __name__ == '__main__':
    robot1 = CoCube(3, enable_return=False)
    robot2 = CoCube(4, enable_return=False)
    robot3 = CoCube(5, enable_return=False)
    robot4 = CoCube(6, enable_return=False)
    while True:
        for x in range(1, 6):
            for y in range(1, 6):
                robot1.process_data(block="0", func="[display:mbPlot]", params=[x, y])
                robot2.process_data(block="0", func="[display:mbPlot]", params=[x, y])
                robot3.process_data(block="0", func="[display:mbPlot]", params=[x, y])
                robot4.process_data(block="0", func="[display:mbPlot]", params=[x, y])
                time.sleep(0.1)
        for x in range(1, 6):
            for y in range(1, 6):
                robot1.process_data(block="0", func="[display:mbUnplot]", params=[x, y])
                robot2.process_data(block="0", func="[display:mbUnplot]", params=[x, y])
                robot3.process_data(block="0", func="[display:mbUnplot]", params=[x, y])
                robot4.process_data(block="0", func="[display:mbUnplot]", params=[x, y])
                time.sleep(0.1)


