<div align="center">

# PX4-NeuPilot

**PX4-Autopilot for Neural Contorl**


[![PX4](https://img.shields.io/badge/PX4-Autopilot-orange)](https://px4.io/)
[![License](https://img.shields.io/badge/License-BSD--3-green.svg)](LICENSE)

</div>

## Quick Start
### 📦 Clone the repo
```bash
git clone https://github.com/Arclunar/PX4-Neupilot.git --recursive --depth 1
```


### 🎯 Docker Deployment
#### 0. Adjust proxy settings
Before building the docker image, you need to change the proxy settings in the dockerfile : ``docker/px4-simulation.dockerfile``
```
ENV http_proxy=http://127.0.0.1:7890 \
    https_proxy=http://127.0.0.1:7890 \
    HTTP_PROXY=http://127.0.0.1:7890 \
    HTTPS_PROXY=http://127.0.0.1:7890 \
    no_proxy=localhost,127.0.0.1,::1 \
    NO_PROXY=localhost,127.0.0.1,::1
```
#### 1. Build the docker image
```bash
just build-px4
```

#### 2. Then run docker
```bash
just run-px4
```

#### 3. Start the simulation
In the docker terminal, run
```bash
runsim.sh 2
```
to start a gazebo simulation with a quadrotor , with Micro-XRCED-DDS agent running.

---
### For interact with ros2
see [PX4-ROS2-Bridge](https://github.com/Arclunar/PX4-ROS2-Bridge/tree/main#)


---
<div align="center">

**如果这个项目对你有帮助，请给一个 ⭐ Star！**

Made with ❤️ by Arclunar

</div>
