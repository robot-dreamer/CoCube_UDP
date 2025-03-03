# Quick Start

CoCube supports two wireless data transfer methods: Bluetooth and Wi-Fi. To ensure the stability of multi-agent algorithm deployment, we use the UDP protocol in Python to establish a connection with CoCube.Therefore, a router is required for data transfer within the local network.

## First Step： Flash .ubp file onto CoCube
When you get the CoCube, the first step is to flash the basic interface program onto it. We use [MicroBlocks](https://microblocks.fun/run/microblocks.html) as the flashing platform. There are two ways to connect to the CoCube：USB or BLE. Once the connection is successful，you can drag the [wifi_udp.ubp](wifi_udp.ubp) file to the website. The file will then be flashed onto CoCube. Note that there is a blue block in Fig.1, which contains the related Wi-Fi connection parameters and should be modified according to your setup.
![fig.1](/pic/microblocks_param.png)
<center> Fig.1 </center>
You should replace the blank line after “WiFi” with the name of your Wi-Fi."SSID" is the corresponding password. "gateway" is the gateway of your router. "hostip" is the IP
 address of your computer.(So make sure your computer and CoCubes are in the same local area network) After getting the computer's IP, CoCube also needs to set up its own IP to communicate with the computer.“ip_prefix” is the prefix of CoCube's own IP, it is added with ‘ID’ to get CoCube's IP in current LAN. For example in Fig.1, its IP is "192.168.31.102". Finally, add "port_prefix" and "ID" to get the port number of CoCube, which is the default setting. After the modification is completed, click the block to update the parameters.

## Second Step: Setup the environment in the computer
First ensure that the parameters in [yaml file](config.yml) and CoCube are synchronized.
Then create a conda environment.
> cd to/this/path

> conda create --name cocube python==3.11

> conda activate cocube

> pip install -r requirements.txt

> git clone https://github.com/sybrenstuvel/Python-RVO2.git

> cd Python-RVO2 

> python setup.py build

> python setup.py install

## Third Step: Start Up！
We offer three demonstrations to showcase CoCube's simplicity, ease of use, and powerful clustering capabilities.

1. In [follow your leader](cocube_follow_leader.py) n CoCubes make a small train, Every CoCube will regard the previous CoCube as its leader and keep following it.
2. In [munkres](cocube_munkres.py), You can set up some targets, The munkres algorithm will calculate the minimum cost and assign the shortest path to each CoCube.[munkres for ICRA](cocube_munkres_for_icra.py) uses 32 CoCubes to make the formation of the letters “ICRA 2025”.
3. In [RVO2](cocube_rvo2.py). Even number of CoCubes will swap positions in pairs, while Avoiding collisions.

# License
cocube is distributed under the terms of the [MIT](https://spdx.org/licenses/MIT.html) license.