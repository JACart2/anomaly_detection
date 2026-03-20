#!/bin/bash

# Install dependencies for the buffer model
pip install -r requirements.txt --break-system-packages

apt-get update
apt-get install -y wget

# Set a temporary download directory inside the container
TMP_DIR="/tmp/cudnn_download"
mkdir -p "$TMP_DIR"

# Download cuDNN .deb to the temp directory
wget -P "$TMP_DIR" https://developer.download.nvidia.com/compute/cudnn/9.19.1/local_installers/cudnn-local-repo-ubuntu2404-9.19.1_1.0-1_amd64.deb

# Install the local repository
sudo dpkg -i "$TMP_DIR"/cudnn-local-repo-ubuntu2404-9.19.1_1.0-1_amd64.deb

# Copy keyring
sudo cp /var/cudnn-local-repo-ubuntu2404-9.19.1/cudnn-*-keyring.gpg /usr/share/keyrings/

# Update package lists and install cuDNN
sudo apt-get update
sudo apt-get -y install cudnn9-cuda-12

# Optionally clean up the downloaded file
rm -rf "$TMP_DIR"