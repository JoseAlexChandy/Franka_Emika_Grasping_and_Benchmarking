#!/bin/bash

# Check if a numerical parameter is provided
if [ $# -eq 0 ]; then
  echo "Please provide an algorithm number as a parameter."
  echo "0: DexNet, 1: 6DGraspNet, 2: GPD, 3: ContactGraspNet, 4: GrConvNet"
  exit 1
fi

# Algorithm array
declare -a algorithms=(
  "dexnet_container heap/benchmark_dexnet"
  "6dgraspnet_container heap/benchmark_6dgraspnet"
  "gpd_container heap/benchmark_gpd"
  "contactgraspnet_container heap/benchmark_contactgraspnet"
  "grconvnet_container heap/benchmark_grconvnet"
)

# Extract the selected algorithm's details
selected_algorithm=${algorithms[$1]}
IFS=' ' read -ra ALG_DETAILS <<< "$selected_algorithm"
CONTAINER_NAME="${ALG_DETAILS[0]}"
IMAGE_NAME="${ALG_DETAILS[1]}"

# Launch Camera
gnome-terminal -- bash -c "roslaunch realsense2_camera rs_rgbd.launch enable_pointcloud:=true tf_prefix:=measured/camera; exec bash"
sleep 5  # Give some time for the camera to initialize

# Define GPU machine's IP
GPU_MACHINE="172.22.3.8"

# Execute commands on the remote machine via SSH
gnome-terminal -- bash -c "ssh -t -Y $GPU_MACHINE '
  echo \"Connected to \$GPU_MACHINE\";
  cd ~/grasping-benchmarks-panda-1/docker && echo \"Changed directory\";
  bash run.sh heap $CONTAINER_NAME $IMAGE_NAME;
  bash
'"

# SL activation command (without ROS setup)
SL_ACTIVATE_CMD="source \$(conda info --base)/etc/profile.d/conda.sh && conda activate grasp_benchmark"

# Run SL in a new terminal
gnome-terminal -- bash -c "$SL_ACTIVATE_CMD && cd ~/GraspBenchmarkWorkspace/sl/build/sl_panda && ./xrpanda; exec bash"
sleep 10

# Commands to activate the conda environment and source ROS setup
ACTIVATE_CMD="source \$(conda info --base)/etc/profile.d/conda.sh && conda activate grasp_benchmark && source ~/GraspBenchmarkWorkspace/devel/setup.bash"

# Run DetectBox_.py for ARUCO marker detection
gnome-terminal -- bash -c "$ACTIVATE_CMD && rosrun marker_detect DetectBox_.py; exec bash"

# Run Pix2Depth.py for deprojection
gnome-terminal -- bash -c "$ACTIVATE_CMD && rosrun marker_detect Pix2Depth.py; exec bash"

# Simulation command
gnome-terminal -- bash -c "$ACTIVATE_CMD && python ~/GraspBenchmarkWorkspace/Benchmark/grasp_benchmark/simulate_from_real.py '057_racquetball' '/home/heap/GraspBenchmarkWorkspace/YCB/models/ycb/'; exec bash"
sleep 5

# Run with_aruco.py script
gnome-terminal -- bash -c "$ACTIVATE_CMD && python ~/GraspBenchmarkWorkspace/Benchmark/grasp_benchmark/with_aruco.py 4.10; exec bash"

