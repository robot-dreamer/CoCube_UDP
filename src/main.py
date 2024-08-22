from src.cocube_udp import CoCube

import time
import random

if __name__ == '__main__':
    robot1 = CoCube(1, enable_return=False)
    while True:
        x = random.randint(1, 5)
        y = random.randint(1, 5)
        robot1.process_data(block="0", func="[display:mbPlot]", params=[x, y])
        time.sleep(0.1)
        robot1.process_data(block="0", func="[display:mbUnplot]", params=[x, y])
        time.sleep(0.1)


