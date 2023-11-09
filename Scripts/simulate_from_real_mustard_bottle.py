import os

import rospy
from geometry_msgs.msg import Pose

from grasp_benchmark.utils.ros_utils import *
from alr_sim.sims.SimFactory import SimRepository


from utils.ycb_utils import YCBLoader

NUMBER_OF_CANDIDATES = 1

global_scene = None

def main():
    # ROS Init
    rospy.init_node("test")
    pub_grasp_pose = rospy.Publisher('grasp_pose', Pose, queue_size=10)
    grasp_pose = Pose()

    factory_string = "mj_beta"
    YCB_loader = YCBLoader(ycb_base_folder="/home/heap/GraspBenchmarkWorkspace/YCB/models/ycb/", factory_string=factory_string)
    
    print('waiting for pose of the brick')
    pose_msg = rospy.wait_for_message('brick_pose', Pose)
    print(pose_msg)
    q_WB = [pose_msg.orientation.w, 
            pose_msg.orientation.x, 
            pose_msg.orientation.y,
            pose_msg.orientation.z]
    rBW_W = [pose_msg.position.x, pose_msg.position.y, -0.017]
    print('received position:', rBW_W)
    print('received orientation:', q_WB)
    mustard, mustard_name = YCB_loader.get_ycb_object(obj_id="006_mustard_bottle", pos=rBW_W, quat=q_WB)

    # Initializing
    object_list = [mustard]
    
    
    # Generate the chosen Scene and Agent
    sim_factory = SimRepository.get_factory(factory_string)
    scene = sim_factory.create_scene(object_list=object_list, dt=0.0003)
    agent = sim_factory.create_robot(scene, dt=0.0003)

    agent.inhand_cam.set_cam_params(width=640, height=480)

    scene.start()

    print('mustard position')
    print(scene.get_obj_pos(obj_name = mustard_name))


    

    target_obj_id = scene.get_obj_seg_id(obj_name=mustard_name)
  
    print('for mustard:')
    print(scene.get_obj_seg_id(obj_name=mustard_name))


    wait_time = 0.5

    # Go to home position
    home_pos = [0.35, 0.0, 0.60]
    home_or = [0.0, 1.0, 0.0, 0.0]
    agent.receiveState()
    agent.robot_logger.max_time_steps = 100000000
    agent.gotoCartPositionAndQuat(home_pos, home_or, duration=12)
    print('home done.......................')
    agent.receiveState()
    print('current position:')
    print(agent.current_c_pos)
    print('current orientation:')
    print(agent.current_c_quat)
    

    print('final mustard position')
    print(scene.get_obj_pos(obj_name = mustard_name))

    print('----------------------------')
    print('select grasping algorithm:\n1. DEXNET \n2. GRASPNET \n3. GPD\n')
    algorithm_number = input()
    if int(algorithm_number) == 1:
        print('calling Dexnet...')
        algorithm_name = "/dexnet_bench/dexnet_grasp_planner_service"
    elif int(algorithm_number) == 2:
        print('calling Graspnet...')
        algorithm_name = "/graspnet_bench/graspnet_grasp_planner_service"
    else: 
        print('calling GPD...')
        algorithm_name = "/gpd_bench/gpd_grasp_planner/gpd_grasp_planner_service"

    # 2. Calling the grasp planner
    result_from_service = call_grasp_planner(cam=agent.inhand_cam, 
                                            service_id=algorithm_name, 
                                            seg_obj_id=target_obj_id, 
                                            num_of_candidates=NUMBER_OF_CANDIDATES)
    print(result_from_service)
    grasp_pos = result_from_service[0]['position']
    grasp_quat = result_from_service[0]['quat'] 


    # execute grasp
    agent.set_gripper_width = 0.08
    agent.wait(wait_time)
    k_grasp = quat2mat(grasp_quat)[:,2]
    target_pos = grasp_pos - 0.05*k_grasp
    agent.gotoCartPositionAndQuat(target_pos, grasp_quat, duration=12)
    agent.wait(wait_time)
    target_pos = grasp_pos - 0.005*k_grasp
    agent.gotoCartPositionAndQuat(target_pos, grasp_quat, duration=3)
    agent.receiveState()
    print('\n actual position at grasp:')
    print(agent.current_c_pos)
    print('actual orientation at grasp:')
    print(quat2mat(agent.current_c_quat))
    print('\n')
    agent.wait(wait_time)
    agent.close_fingers(duration=2)
    print('-------- FINGERS CLOSED')
    agent.wait(wait_time)
    agent.gotoCartPositionAndQuat(home_pos, home_or)
    agent.wait(5)
    agent.set_gripper_width = 0.08
    agent.wait(3)

    to_real = input('execute in real robot? (y/n)')
    if to_real == 'y':
        grasp_pose.position.x = grasp_pos[0] 
        grasp_pose.position.y = grasp_pos[1] 
        grasp_pose.position.z = grasp_pos[2]+(0.017-0.005) #compensation due to table level
        grasp_pose.orientation.w = grasp_quat[0]
        grasp_pose.orientation.x = grasp_quat[1]
        grasp_pose.orientation.y = grasp_quat[2]
        grasp_pose.orientation.z = grasp_quat[3]
        for i in range(100):
            pub_grasp_pose.publish(grasp_pose)


if __name__ == "__main__":
    main()




