#!/bin/bash

set -e  # Exit immediately if a command exits with a non-zero status

# Function to check if the last command was successful
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error: $1 failed. Exiting."
        exit 1
    fi
}

echo "Starting Docker and NVIDIA driver installation script..."

# Step 1: Set up the Docker repository
echo "Setting up the Docker repository..."

# Update the package index
echo "Updating the package index..."
sudo apt-get update -y
check_success "Package index update"

# Install required packages for Docker
echo "Installing prerequisites for Docker..."
sudo apt-get install -y \
    apt-transport-https \
    ca-certificates \
    curl \
    software-properties-common \
    gnupg
check_success "Prerequisite installation"

# Add Docker's official GPG key
echo "Adding Docker's GPG key..."
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
check_success "Adding Docker's GPG key"

# Set up the Docker stable repository
echo "Adding Docker's stable repository..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
check_success "Setting up Docker's stable repository"

# Step 1.2: Install Docker Engine
echo "Installing Docker Engine..."
sudo apt-get update -y
check_success "Package index update for Docker installation"
sudo apt-get install -y docker-ce docker-ce-cli containerd.io
check_success "Docker installation"

# Verify Docker installation
echo "Verifying Docker installation..."
sudo docker --version
check_success "Docker verification"

# Start and enable Docker service
echo "Starting and enabling Docker service..."
sudo systemctl start docker
check_success "Starting Docker service"
sudo systemctl enable docker
check_success "Enabling Docker service"

# Step 2: Install NVIDIA drivers and nvidia-container-toolkit
echo "Installing NVIDIA drivers and nvidia-container-toolkit..."

# Add NVIDIA package repositories
echo "Adding NVIDIA's package repositories..."
distribution=$(. /etc/os-release; echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
check_success "Adding NVIDIA GPG key"
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
check_success "Adding NVIDIA container repository"

# Update package lists
sudo apt-get update -y
check_success "Updating package lists for NVIDIA"

# Install NVIDIA drivers
# echo "Installing NVIDIA driver..."
# sudo apt-get install -y nvidia-driver-470  # Change version as per your requirements
# check_success "NVIDIA driver installation"

# Install nvidia-container-toolkit
echo "Installing NVIDIA Container Toolkit..."
sudo apt-get install -y nvidia-container-toolkit
check_success "NVIDIA Container Toolkit installation"

# Configure NVIDIA runtime
echo "Configuring NVIDIA runtime..."
sudo nvidia-ctk runtime configure --runtime=docker
check_success "Configuring NVIDIA runtime"

sudo usermod -aG docker $USER
newgrp docker

# Restart Docker to apply changes
echo "Restarting Docker service..."
sudo systemctl daemon-reload && sudo systemctl restart docker

check_success "Restarting Docker service"

# Step 3: Verify NVIDIA drivers
#echo "Verifying NVIDIA driver installation..."
#if command -v nvidia-smi &> /dev/null; then
#    nvidia-smi
#    echo "NVIDIA driver installation and verification successful!"
#else
#    echo "Error: nvidia-smi command not found! NVIDIA driver installation failed!"
#    exit 1
#fi

echo "Docker installation completed successfully!"
