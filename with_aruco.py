import os
import sys

from grasp_benchmark.utils.ros_utils import *
from grasp_benchmark.utils.ros_utils_real import *
from alr_sim.sims.SimFactory import SimRepository
from alr_sim.utils.geometric_transformation import mat2quat

import numpy as np

from utils.ycb_utils import YCBLoader

# ROS Imports
import ros_numpy
import rospy
from rospy import Publisher
from geometry_msgs.msg import Pose, TransformStamped, Point
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header

NUMBER_OF_CANDIDATES = 1

global_scene = None


centreC = np.array([0.0, 0.0, 0.0])
corner0 = np.array([0.0, 0.0, 0.0])
forwardO = np.array([0.0, 0.0, 0.0])

def centreCallback(msg):
    global centreC
    centreC = np.array([msg.x, msg.y, msg.z])

def corner0Callback(msg):
    global corner0
    corner0 = np.array([msg.x, msg.y, msg.z])

def forwardCallback(msg):
    global forwardO
    forwardO = np.array([msg.x, msg.y, 0.0])
    forwardO = forwardO/np.linalg.norm(forwardO)


#////////////////////////////////////////////////////////////////////////////////

def main():

    # ROS Init
    rospy.init_node('simplified_node_aruco', anonymous=True)

    # subscribe to aruc detection
    sub_centre = rospy.Subscriber('centre', Point, centreCallback)
    sub_corner0 = rospy.Subscriber('corner0', Point, corner0Callback)
    sub_forward = rospy.Subscriber('forward', Point, forwardCallback)
    pub_pose = rospy.Publisher('brick_pose', Pose, queue_size=10)
    brick_pose = Pose()

    # initialize robot
    sim_factory = SimRepository.get_factory("sl")
    s = sim_factory.create_scene()
    robot = sim_factory.create_robot(s, robot_name="bob")
    s.start()
    robot.use_inv_dyn = False
    wait_time = 0.5

    # Go to home position
    home_pos = [0.35, 0.0, 0.60]
    home_or = [0.0, 1.0, 0.0, 0.0]
    robot.receiveState()
    robot.robot_logger.max_time_steps = 100000000
    robot.gotoCartPositionAndQuat(home_pos, home_or, duration=12)
    print('home done.......................')
    robot.receiveState()
    print('current position:')
    print(robot.current_c_pos)
    print('current orientation:')
    print(robot.current_c_quat)


    # from Pybullet and sheet:
    # rPB_B = np.array([0.007, -0.02, 0.0])
    # rPA_A = np.array([-0.15, 0, 0])
    # RAB = np.array([[0.707106781, -0.707106781, 0],
    #                 [0.707106781, 0.707106781, 0],
    #                 [0, 0, 1]])

    rBA_A = np.array([0,-0.10, 0.0])
    RAB = np.eye(3)
    
    
    
    print('press key when photo is ready')
    input()

    # publish pose of brick 100 times:
    for i in range(100):
        #rPC_C = 0.001*centreC
        rAC_C = 0.001*corner0
        rCW_W, RWC = get_cam_pose(robot.current_c_pos, robot.current_c_quat)
        rAW_W = rCW_W + np.matmul(RWC, rAC_C)
        iA_W = forwardO
        jA_W = np.cross(np.array([0,0,1]), iA_W)
        RWA = np.transpose(np.array([iA_W,jA_W,[0,0,1]]))
        RWB = np.matmul(RWA, RAB)
        qWB = mat2quat(RWB)
        #rBW_W = rAW_W + np.matmul(RWA, rPA_A) - np.matmul(RWB, rPB_B) 
        rBW_W = rAW_W + np.matmul(RWA, rBA_A)
        print('---------------------------------')
        print('rAW_W: ', rAW_W)
        print('RWA: ', RWA)
        print('rBW_W: ', rBW_W)
        print('RWB: ', RWB)
        print('qWB: ', qWB)
        brick_pose.position.x = rBW_W[0] 
        brick_pose.position.y = rBW_W[1] 
        brick_pose.position.z = rBW_W[2]
        brick_pose.orientation.w = qWB[0]
        brick_pose.orientation.x = qWB[1]
        brick_pose.orientation.y = qWB[2]
        brick_pose.orientation.z = qWB[3]
        pub_pose.publish(brick_pose)

    print('waiting for grasp pose from simulation')
    grasp_pose_msg = rospy.wait_for_message('grasp_pose', Pose)
    grasp_quat = [grasp_pose_msg.orientation.w, 
                grasp_pose_msg.orientation.x, 
                grasp_pose_msg.orientation.y,
                grasp_pose_msg.orientation.z]
    grasp_pos = [grasp_pose_msg.position.x, 
                grasp_pose_msg.position.y, 
                grasp_pose_msg.position.z]
    print('received grasp position:', grasp_pos)
    print('received grasp orientation:', grasp_quat)

    # execute grasp
    robot.set_gripper_width = 0.08
    robot.wait(wait_time)
    k_grasp = quat2mat(grasp_quat)[:,2]
    target_pos = grasp_pos - 0.05*k_grasp
    robot.gotoCartPositionAndQuat(target_pos, grasp_quat, duration=12)
    robot.wait(wait_time)
    target_pos = grasp_pos - 0.005*k_grasp
    robot.gotoCartPositionAndQuat(target_pos, grasp_quat, duration=3)
    robot.receiveState()
    print('\n actual position at grasp:')
    print(robot.current_c_pos)
    print('actual orientation at grasp:')
    print(quat2mat(robot.current_c_quat))
    print('\n')
    robot.wait(wait_time)
    robot.close_fingers(duration=2)
    print('-------- FINGERS CLOSED')
    robot.wait(wait_time)
    robot.gotoCartPositionAndQuat(home_pos, home_or)
    robot.wait(5)
    robot.set_gripper_width = 0.08
    robot.wait(3)

if __name__ == "__main__":
    main()





# CAMERA on BOB-Control: 
#   roslaunch realsense2_camera rs_rgbd.launch enable_pointcloud:=true tf_prefix:=measured/camera
#   
# GRASP SERVICE in vincent-gpu:
#   ssh -Y pablolopezcustodio@10.0.2.10
#   cd grasping-benchmarks-panda/docker
#   bash run.sh heap dexnet_container heap/benchmark_dexnet 
#   or:  bash run.sh heap dexnet_container
#   source /workspace/catkin_ws/devel/setup.bash
#   export ROS_MASTER_URI=http://10.0.2.3:11311
#   export ROS_IP=10.0.2.10
#   roslaunch grasping_benchmarks_ros grasp_planning_benchmark.launch
#
# DETECT ARUCO MARKER in Bob-control:
#   conda activate grasp_benchmark
#   source ~/GraspBenchmarkWorkspace/devel/setup.bash
#   rosrun marker_detect DetectBox_.py
#
# DEPROJECTION in bob-control:
#   source ~/GraspBenchmarkWorkspace/devel/setup.bash
#   rosrun marker_detect Pix2Depth.py
#
# SIMULATION
#   conda activate grasp_benchmark
#   source ~/GraspBenchmarkWorkspace/devel/setup.bash
#   python ~/GraspBenchmarkWorkspace/Benchmark/grasp_benchmark/simulate_from_real.py
#
# SL
#   cd ~/sl_ws/sl/build/sl_panda
#   Activate FCI in the franka emika interface
#   ./xrpanda
#
# SCRIPT
#   conda activate grasp_benchmark
#   source ~/GraspBenchmarkWorkspace/devel/setup.bash
#   python ~/GraspBenchmarkWorkspace/Benchmark/grasp_benchmark/with_aruco.py
#
