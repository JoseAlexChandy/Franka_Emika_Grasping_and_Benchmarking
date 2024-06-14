#!/bin/bash

# Check if a numerical parameter is provided
if [ $# -eq 0 ]; then
  echo "Please provide an algorithm number as a parameter."
  echo "0: DexNet, 1: 6DGraspNet, 2: GPD, 3: ContactGraspNet, 4: GrConvNet"
  exit 1
fi

# Algorithm array
declare -a algorithms=(
  "dexnet_container heap/benchmark_dexnet dexnet"
  "6dgraspnet_container heap/benchmark_6dgraspnet 6dgraspnet"
  "gpd_container heap/benchmark_gpd gpd"
  "contactgraspnet_container heap/benchmark_contactgraspnet contactgraspnet"
  "grconvnet_container heap/benchmark_grconvnet grconvnet"
)

# Extract the selected algorithm's details
selected_algorithm=${algorithms[$1]}
IFS=' ' read -ra ALG_DETAILS <<< "$selected_algorithm"
CONTAINER_NAME="${ALG_DETAILS[0]}"
IMAGE_NAME="${ALG_DETAILS[1]}"
ALGORITHM_NAME="${ALG_DETAILS[2]}"

# CAMERA AND SL ON BOB
# Launch Camera
gnome-terminal -- bash -c "roslaunch realsense2_camera rs_rgbd.launch enable_pointcloud:=true tf_prefix:=measured/camera; exec bash"
sleep 5 # Give some time for the camera to initialize

# Define GPU machine's IP
GPU_MACHINE="172.22.3.8"

# In BOB-Control for SL
gnome-terminal -- bash -c "source \$(conda info --base)/etc/profile.d/conda.sh && conda activate grasp_benchmark && cd ~/GraspBenchmarkWorkspace/sl/build/sl_panda && ./xrpanda && echo 'Type \"st\" and select 6'; exec bash"
sleep 10

# SEGMENTATION ON SERVER
# SSH and run segmentation
gnome-terminal -- bash -c "ssh -t -Y 172.22.3.7 '
  cd segmentation
  docker compose run node
  '"

# VISUALIZE SEGMENTATION ON BOB
# Visualize segmentation results
gnome-terminal -- bash -c "rosrun image_view image_view image:=/seg_image; exec bash"

# Execute algorithm-specific command on the remote machine via SSH
gnome-terminal -- bash -c "ssh -t -Y $GPU_MACHINE '
  echo \"Connected to \$GPU_MACHINE\";  # Confirm connection
  cd ~/grasping-benchmarks-panda-1/docker && echo \"Changed directory\";  # Change directory and confirm
  bash run.sh heap $CONTAINER_NAME $IMAGE_NAME;  # Run your script
  bash  # Keep the terminal open for interactive use
'"

# Run simplified_docker_real.py script with algorithm-specific parameter
gnome-terminal -- bash -c "source \$(conda info --base)/etc/profile.d/conda.sh && conda activate grasp_benchmark && source ~/GraspBenchmarkWorkspace/devel/setup.bash && python ~/GraspBenchmarkWorkspace/Benchmark/grasp_benchmark/simplified_docker_real.py $ALGORITHM_NAME; exec bash"

