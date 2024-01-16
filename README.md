# MEC 202 - Industrial Awareness and Group Project

## Remote Robot Control Demo Project - Draft Scripts

 Before you start, please make sure that you have fulfilled the following requirements.

1. The robot already has Ubuntu installed. (Ubuntu 22.04 LTS is recommended.)
2. Your PC already has a workable Ubuntu OS. (An installed OS or a virtual machine is OK, but WSL is not acceptable.)

## Content

1. [Installing ROS2](#1)
2. [Getting familiar with ROS2 key features](#2)
3. [Controlling the vehicle within LAN](#3)
4. [Controlling the vehicle within WAN](#4)

<h3 id="1">1. Installing ROS2</h3>

ROS2 (Robot Operating System 2) is an open-source software framework designed for robotics. It is the successor to ROS (Robot Operating System) and offers a flexible framework for writing robot software. The data transfer between the robot and the PC in this project relies on one of the core components of ROS2, **DDS (Data Distribution Service)**.

Here are the steps to install ROS2 Humble on Ubuntu 22.04 LTS. You must complete these steps on both the robot and the PC. Either Method 1 or Method 2 can be selected.

- **Method 1: Using installation scripts.**

  Use scripts from https://github.com/fishros/install.

  Open the terminal and execute

  ```bash
  wget http://fishros.com/install -O fishros && . fishros
  ```

  <img width="741" alt="01" src="https://github.com/Lya-M1RA/MEC202_AY23-24_remote_control_demo/assets/36181581/2470277f-f388-4203-a703-7f4b89084f52">

  Select the following options, 

  ```bash
  [1]:一键安装(推荐):ROS(支持ROS/ROS2,树莓派Jetson)
  
  [1]:更换系统源再继续安装
  
  [2]:更换系统源并清理第三方源
  
  [1]:humble(ROS2)
  
  [1]:humble(ROS2)桌面版
  ```

  the installation will automatically start.

- **Method 2: Manually Installation**

  **Change Ubuntu software sources (https://mirrors.tuna.tsinghua.edu.cn/help/ubuntu/)**

  ```bash
  sudo gedit /etc/apt/sources.list
  ```

  In the pop-up text editor interface, replace the original content with the following content

  ```bash
  # 默认注释了源码镜像以提高 apt update 速度，如有需要可自行取消注释
  deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy main restricted universe multiverse
  # deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy main restricted universe multiverse
  deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-updates main restricted universe multiverse
  # deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-updates main restricted universe multiverse
  deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-backports main restricted universe multiverse
  # deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-backports main restricted universe multiverse
  
  deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-security main restricted universe multiverse
  # deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-security main restricted universe multiverse
  
  # 预发布软件源，不建议启用
  # deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-proposed main restricted universe multiverse
  # # deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-proposed main restricted universe multiverse
  ```

  Save and close the window.

  **Use ROS2 repository mirror from TUNA (https://mirrors.tuna.tsinghua.edu.cn/help/ros2/)**

  ```bash
  sudo apt update
  sudo apt install curl gnupg2
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt update
  ```

  **Install ROS2 Humble Desktop**

  ```bash
  sudo apt install ros-humble-desktop-full
  ```

  **Install other needed packages**

  ```bash
  sudo apt install python3-colcon-common-extensions python3-argcomplete python3-rosdep
  ```

  **Use rosdistro mirror fron TUNA (https://mirrors.tuna.tsinghua.edu.cn/help/rosdistro/)**

  ```bash
  # 手动模拟 rosdep init
  sudo mkdir -p /etc/ros/rosdep/sources.list.d/
  sudo curl -o /etc/ros/rosdep/sources.list.d/20-default.list https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/sources.list.d/20-default.list
  # 为 rosdep update 换源
  export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml
  rosdep update
  
  # 每次 rosdep update 之前，均需要增加该环境变量
  # 为了持久化该设定，可以将其写入 .bashrc 中，例如
  echo 'export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml' >> ~/.bashrc
  ```

  **Setup PATH**

  ```bash
  source /opt/ros/humble/setup.bash
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc 
  ```

Thus, the installation of ROS2 and related tools is complete.

<h3 id="2">2. Getting familiar with ROS2 key features</h3>

Before attempting to deploy the vehicle remote control demo, you need to follow few articles to understand some of the key features of ROS2.

- **Using `turtlesim`, `ros2`, and `rqt`**

  https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html

- **Understanding nodes**

  https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html#

- **Understanding topics**

  https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html

- **Understanding parameters**

  https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html

Please complete the contents of the above tutorials. For a systematic introduction to ROS2, you can refer to the tutorials on these websites.

- (Chinese) ROS2入门教程 https://book.guyuehome.com/
- (English) ROS2 Documentation https://docs.ros.org/en/humble/Tutorials.html

<h3 id="3">3. Controlling the vehicle within LAN</h3>

Vehicle control via a PC can be divided into two parts as follows

- **Controller signals**

  A controllerThe PC is connected to a controller. The PC needs to send input signals from the controller to the vehicle.

- **Camera data**

  The vehicle is equipped with a camera. The vehicle needs to send the images captured by the camera to the PC to provide a live view of the vehicle.

You are required to complete the following 2 steps on both the PC and the vehicle controller.

**Download the source code**

```bash
cd ~
sudo apt install git
git clone https://github.com/Lya-M1RA/MEC202_AY23-24_remote_control_demo.git
```

**Compile the source code**

```bash
cd ~/rc_demo
rosdep install --from-paths src --ignore-src -r -y
colcon build
echo "source ~/rc_demo/install/setup.bash" >> ~/.bashrc
```

So far you have downloaded the code needed for this project and built them in a ROS2 workspace.

Make sure the vehicle controller is connected to the same LAN (Wi-Fi network) as your PC **(use your own router or a cellular hotspot; school Wi-Fi `XJTLU` disables clients on the LAN from communicating with each other)**. Then you can initiate remote control of your vehicle with the following commands.

**On your PC**

```bash
ros2 launch rc_demo pc_launch.py
```

**On the vehicle controller**

```bash
ros2 launch rc_demo vehicle_launch.py
```

In the above commands, `pc_launch.py` launches two nodes. The `/joy_node` node, captures the signals from the controller's buttons, triggers and joysticks and publishes them as the `/joy` topic. The `/rqt_image_view` node subscribes to the `/image_raw/theora` topic from the vehicle and displays the video stream from the topic.

`vehicle_launch.py` also launches two nodes. The `/manual_mode` node, subscribes to the `/joy` topic from the PC. This node updates the vehicle's state, target actions, and other information based on the raw input signals from the controller and sends commands to the vehicle's motor controllers. The `/usb_cam_node_exe` node encodes the footage from the camera on the vehicle into a video stream and publishes it to the `/image_raw/theora` topic.

With the above commands, you should be able to see the image coming from the vehicle on your PC and control the movement of the vehicle using a controller connected to your PC.

<h3 id="4">4. Controlling the vehicle within WAN</h3>

We have implemented the control of the vehicle through LAN communication in the previous section. However, in real scenarios if the vehicle is traveling autonomously outdoors, how to accomplish remote control of the vehicle?

The vehicle is connected to the internet via a 4G/5G wireless terminal device, which means we can try to control the vehicle using communications over the internet. However, customizing a set of protocols for data exchange seems to be a bit complicated; there is a simple idea for implementation without considering the needs of connection stability, data security, etc. 

One can try to connect the vehicle to the same virtual LAN as the PC via a virtual private network (VPN) or similar technology so that no code changes are required, and the ROS2 communication framework still works. Here's a specific approach that works.

**ZeroTier network**

We need to use the ZeroTier service. ZeroTier is a way to connect devices over your own private network anywhere in the world. A ZeroTier network is essentially a secure Local Area Network (LAN) that you can use anywhere in the world.

- **Initialize a ZeroTier network**

  Go to [my.zerotier.com](https://my.zerotier.com/) and create an account. Then click "Create A Network".

  <img width="986" alt="02" src="https://github.com/Lya-M1RA/MEC202_AY23-24_remote_control_demo/assets/36181581/be4e0b8e-5884-4a88-9544-de2d4c3ab500">

  This creates a virtual network with a random ID and a random name. For example I got "distracted_joybubbles" and `35c192ce9b0787a5` here.

  <img width="798" alt="03" src="https://github.com/Lya-M1RA/MEC202_AY23-24_remote_control_demo/assets/36181581/1265ce82-9ec1-44d9-bb53-f7be450f514e">

  Click anywhere on the network to go to the details page for this network. You can change the name of your network.

- **Install ZeroTier client**

  On both the PC and the vehicle controller

  ```bash
  curl -s https://install.zerotier.com | sudo bash
  ```

  Let ZeroTier start with Ubuntu

  ```bash
  sudo systemctl enable zerotier-one
  ```

- **Let the PC and the vehicle controller join your ZeroTier network**

  ```bash
  sudo zerotier-cli join <network id>
  ```

  \* Replace `<network id>` with your ZeroTier network ID.

  Refresh your ZeroTier network management page and you'll be able to see new devices appear in `Member` section. Click the `Auth?` checkbox to the left of the item to allow the device to join the network.

  <img width="1159" alt="04" src="https://github.com/Lya-M1RA/MEC202_AY23-24_remote_control_demo/assets/36181581/fb8e4c83-037a-430e-893e-52c29f89d610">

  This gives the PC and the vehicle controller a virtual LAN connection via the ZeroTier network. As long as both have an Internet connection, they can communicate via this ZeroTier network. You can view the network configuration of the device by using the `ip addr` command.


  <img width="1114" alt="05" src="https://github.com/Lya-M1RA/MEC202_AY23-24_remote_control_demo/assets/36181581/a9a69f30-9bf6-489a-90ab-6e3d632cc66e">

  For example, interface 3 in the above figure is the virtual NIC of the ZeroTier service, where the IPv4 address after `inet` is the address of this client in the ZeroTier network. The virtual interfaces of the ZeroTier service are denoted with `zt******`.

After completing the above configuration, have the PC and the vehicle controller run the commands in Section 3 to initiate remote control under different LANs. You should be able to observe similar results to running under the same LAN. However, the connection over the ZeroTier network will have higher latency and jitter compared to the performance under the same LAN.

One of the factors affecting the performance of the ZeroTier network is that the servers used by the ZeroTier network are located outside of mainland China. When the ZeroTier network cannot allow two clients to establish a P2P connection and requires a server to forward the data, the limited bandwidth and high latency of the official servers will result in poor performance.

We can improve this by adding Moon nodes deployed on our own servers to the network, which can replace the official root servers for data forwarding and other functions. Here's how to add Moon nodes to your ZeroTier network. You need to rent a server at a cloud service provider to accomplish this.

**On your server**

- **Install ZeroTier client and join your ZeroTier network**

  ```bash
  curl -s https://install.zerotier.com | sudo bash
  sudo systemctl enable zerotier-one
  sudo zerotier-cli join <network id>
  ```

  Remember to allow the device to join the network via your ZeroTier network management page.

- **Generate a `moon.json` template**

  ```bash
  cd /var/lib/zerotier-one
  sudo zerotier-idtool initmoon identity.public > moon.json
  ```

- **Modify `moon.json`**

  ```bash
  sudo vim moon.json
  ```

  Change `"stableEndpoints": []` to `"stableEndpoints": ["ServerIP/9993"]`, `ServerIP` is the WAN IP address of your server.

- **Generate the `.moon` file**

  ```bash
  sudo zerotier-idtool genmoon moon.json
  ```

- **Move the `.moon` file to `moons.d`**

  ```bash
  sudo mkdir moons.d
  sudo mv 000000xxxxxxxxxx.moon moons.d
  ```

  `000000xxxxxxxxxx.moon` is the file you generated in `/var/lib/zerotier-one`.

- **Restart zerotier-one service**

  ```bash
  sudo systemctl restart zerotier-one
  ```

**On both the PC and the vehicle controller**

```bash
sudo zerotier-cli orbit <moon id> <moon id>
```

Replace `<moon id>` with the ZeroTier network ID of you Moon server.

Use `sudo zerotier-cli listpeers`, you should find a moon node in the list.



This allows the PC and the vehicle controller to communicate through the Moon node. Start the remote control program for the vehicle again and evaluate the performance of the vehicle now remotely controlled in the WAN.
