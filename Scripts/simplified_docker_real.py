import os
import sys

from grasp_benchmark.utils.ros_utils import *
from grasp_benchmark.utils.ros_utils_real import *
from alr_sim.sims.SimFactory import SimRepository


from utils.ycb_utils import YCBLoader

# ROS Imports
import ros_numpy
import rospy
from rospy import Publisher
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header

NUMBER_OF_CANDIDATES = 1

global_scene = None


#////////////////////////////////////////////////////////////////////////////////

def main():

    # grasping services
    if len(sys.argv)!= 2:
        raise Exception("no algorithm specified")
    print('USING ALGORITHM: ', sys.argv[1])
    if sys.argv[1] == 'graspnet':
        service_id = "/graspnet_bench/graspnet_grasp_planner_service"
    elif sys.argv[1] == 'gpd':
        service_id = "/gpd_bench/gpd_grasp_planner/gpd_grasp_planner_service"
    elif sys.argv[1] == 'dexnet':
        service_id = "/dexnet_bench/dexnet_grasp_planner_service"
    else:
        raise Exception("unknown algorithm")

    # ROS Init
    rospy.init_node('simplified_node', anonymous=True)

    # initialize robot
    # Declare robot and take it to home configuration
    sim_factory = SimRepository.get_factory("sl")
    s = sim_factory.create_scene()
    robot = sim_factory.create_robot(s, robot_name="bob")
    s.start()
    robot.use_inv_dyn = False
    wait_time = 0.5

    # 1. Going to home for the start
    home_pos = [0.35, 0.0, 0.50]
    home_or = [0.0, 1.0, 0.0, 0.0]
    robot.receiveState()
    robot.robot_logger.max_time_steps = 100000000
    robot.gotoCartPositionAndQuat(home_pos, home_or, duration=12)
    home_pos = [0.35, 0.0, 0.70]
    home_or = [0.0, 1.0, 0.0, 0.0]
    robot.gotoCartPositionAndQuat(home_pos, home_or, duration=5)
    print('home done.......................')
    robot.receiveState()
    print('current position:')
    print(robot.current_c_pos)
    print('current orientation:')
    print(quat2mat(robot.current_c_quat))

    

    #graspnet_service =      "/graspnet_bench/graspnet_grasp_planner_service"
    #superquadrics_service = "/superquadric_bench/superq_grasp_planner_service"
    #gpd_service =           "/gpd_bench/gpd_grasp_planner/gpd_grasp_planner_service"
    #dexnet_service =        "/dexnet_bench/dexnet_grasp_planner_service"

    # calling service
    print('Press ENTER when photo is ready...')
    input()
    result_from_service = call_grasp_planner_real(ee_position=robot.current_c_pos,
                                                   ee_orientation=robot.current_c_quat,
                                                   service_id=service_id,
                                                   num_of_candidates=NUMBER_OF_CANDIDATES)
    grasp_pos = result_from_service[0]['position']
    grasp_quat = result_from_service[0]['quat']
    print('candidate position:', grasp_pos)
    print('candidate orientation:', quat2mat(grasp_quat))
    print('Press ENTER to grasp...')
    input()

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




