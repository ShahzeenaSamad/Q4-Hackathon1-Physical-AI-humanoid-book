# Installation Guide

Setting up your environment is the first step in your Physical AI journey. This guide covers the installation of the core software stack on **Ubuntu 22.04 LTS (Jammy Jellyfish)**.

---

## 1. Operating System
We recommend **Ubuntu 22.04 LTS**. You can install it as:
*   **Native Installation**: (Best performance) Dual-boot along with Windows or macOS.
*   **WSL 2 (Windows Subsystem for Linux)**: Good for ROS 2 and lightweight simulation. Note that NVIDIA Isaac Sim requires native Linux for GPU acceleration.

---

## 2. ROS 2 Humble Installation
Follow these steps to install the "Humble Hawksbill" version of ROS 2.

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Desktop
sudo apt update
sudo apt install ros-humble-desktop -y

# Source the environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 3. Gazebo Garden Installation
```bash
sudo apt-get update
sudo apt-get install lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-garden -y
```

---

## 4. NVIDIA Isaac Sim Requirements
To run Isaac Sim, you need:
*   **GPU**: NVIDIA RTX 2060 or higher (RTX 3080+ recommended).
*   **Drivers**: NVIDIA Driver version 525+.
*   **Omniverse Launcher**: Download from the NVIDIA website.

---

## 5. Python Environment
We recommend using `venv` or `conda` for managing AI package dependencies (Whisper, LangChain, etc.).

```bash
sudo apt install python3-venv -y
python3 -m venv ~/physical_ai_venv
source ~/physical_ai_venv/bin/activate
pip install openai-whisper langchain rclpy
```
