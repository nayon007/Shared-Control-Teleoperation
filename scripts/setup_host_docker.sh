#!/usr/bin/env bash
# One-time host setup on Ubuntu 20.04: Docker Engine + compose plugin,
# NVIDIA Container Toolkit (GPU for Gazebo), DualSense udev rule.
# ROS itself is NOT installed on the host; it lives in the container.
#
#   bash scripts/setup_host_docker.sh
#   # then log out and back in (docker group)
set -euo pipefail

sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg lsb-release x11-xserver-utils

# --- Docker Engine (official repository) ---------------------------------
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | \
  sudo gpg --dearmor --yes -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io \
  docker-buildx-plugin docker-compose-plugin
sudo usermod -aG docker "$USER"

# --- NVIDIA Container Toolkit ---------------------------------------------
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list > /dev/null
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# --- DualSense: hidraw access for pydualsense --------------------------------
sudo tee /etc/udev/rules.d/70-dualsense.rules > /dev/null <<'EOF'
# Sony DualSense (USB and Bluetooth)
KERNEL=="hidraw*", ATTRS{idVendor}=="054c", ATTRS{idProduct}=="0ce6", MODE="0666"
KERNEL=="hidraw*", KERNELS=="*054C:0CE6*", MODE="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger

echo
echo "Done. Log out and back in (docker group), then check:"
echo "  docker run --rm --gpus all ubuntu:22.04 nvidia-smi"
